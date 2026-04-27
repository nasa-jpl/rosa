#!/usr/bin/env python3.9
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

#  한국어 TurtleSim 에이전트 (경량 + 디버그 로그)
#  현재 rosa 프로젝트의 obstacle, collision, pose 시스템을 모두 반영합니다.

import asyncio
import os
import sys
import threading
import time
from typing import Optional

import dotenv
import rospy
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain.prompts import MessagesPlaceholder
from langchain_core.messages import AIMessage, HumanMessage
from langchain_core.prompts import ChatPromptTemplate
from rich.console import Console
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text

from llm import get_llm

import tools.obstacle as obstacle_tools
from obstacle_store import ObstacleStore
from pose_hub import PoseHub
from pose_logger import (
    POSE_LOG_INTERVAL_SEC,
    CollisionJsonlWriter,
    PoseLogConsumer,
)
from collision_event_sink import make_collision_event_sink
from collision_monitor import CollisionMonitor
from ros_params import get_bool_param
from static_world import load_static_world
import tools.turtle as turtle_tools
from turtle_lifecycle import configure_turtle_lifecycle_listener

from tools.turtle import (
    get_turtle_pose,
    teleport_absolute,
    teleport_relative,
    publish_twist_to_cmd_vel,
    set_pen,
    spawn_turtle,
    kill_turtle,
    reset_turtlesim,
    clear_turtlesim,
    draw_line_segment,
    draw_rectangle,
    draw_polyline,
    draw_circle,
    draw_arc,
    stop_turtle,
    has_moved_to_expected_coordinates,
    calculate_rectangle_bounds,
    check_rectangles_overlap,
)
from tools.obstacle import (
    add_obstacle,
    remove_obstacle,
    list_obstacles,
)

TOOLS = [
    # 거북이 기본 조작
    get_turtle_pose,
    teleport_absolute,
    teleport_relative,
    publish_twist_to_cmd_vel,
    set_pen,
    spawn_turtle,
    kill_turtle,
    stop_turtle,
    reset_turtlesim,
    clear_turtlesim,
    has_moved_to_expected_coordinates,
    # 고수준 그리기
    draw_line_segment,
    draw_rectangle,
    draw_polyline,
    draw_circle,
    draw_arc,
    calculate_rectangle_bounds,
    check_rectangles_overlap,
    # 장애물 CRUD
    add_obstacle,
    remove_obstacle,
    list_obstacles,
]

SYSTEM_PROMPT = (
    "너는 한국어 TurtleSim 에이전트야. 사용자의 한국어 명령으로 거북이를 제어해.\n"
    "항상 한국어로 대답해. 도구를 순차적으로 하나씩 호출해.\n"
    "\n"
    "환경: 11x11 2D 공간. 기본 거북이 turtle1은 중앙(5.544, 5.544).\n"
    "(0,0)=왼쪽아래, (11,11)=오른쪽위. 각도: 0=오른쪽, π/2≈1.57=위, π≈3.14=왼쪽, 3π/2≈4.71=아래.\n"
    "\n"
    "명령 매핑:\n"
    "- 별 → draw_polyline (5꼭짓점 교차연결, closed=True)\n"
    "- 사각형/네모 → draw_rectangle\n"
    "- 원/동그라미 → draw_circle\n"
    "- 삼각형 → draw_polyline (3점, closed=True)\n"
    "- 직선/선 → draw_line_segment\n"
    "- 색상변경 → set_pen (빨강:r=255, 파랑:b=255, 초록:g=255, 노랑:r=255,g=255)\n"
    "- 펜끄기 → set_pen(off=1), 펜켜기 → set_pen(off=0)\n"
    "- 초기화 → reset_turtlesim\n"
    "- 장애물 추가/삭제/목록 → add_obstacle / remove_obstacle / list_obstacles\n"
    "\n"
    "별 그리기: 꼭짓점0→2→4→1→3→0 순서로 연결.\n"
    "크기 미지정시 size=2 사용. 거북이 이름에 /붙이지 마.\n"
    "\n"
    "정적 장애물 맵 (시작 시 자동 로드됨):\n"
    "- wall-south: 남쪽 벽, (0,0)→(11,0) 선분\n"
    "- wall-east: 동쪽 벽, (11,0)→(11,11) 선분\n"
    "- wall-north: 북쪽 벽, (11,11)→(0,11) 선분\n"
    "- wall-west: 서쪽 벽, (0,11)→(0,0) 선분\n"
    "- wet: (5,4)~(7,6) 사각형 영역 — 미끄러운 구간, 가능하면 피해서 이동\n"
    "- a-point: (1,5) 반경 0.25 원형 지점\n"
    "- b-point: (10,5) 반경 0.25 원형 지점\n"
    "- c-point: (6,7) 반경 0.25 원형 지점\n"
    "사용자가 'a-point로 가줘'라고 하면 teleport_absolute(x=1, y=5)로 이동.\n"
    "'wet 피해서 가줘'라고 하면 (5,4)~(7,6) 영역을 우회하는 경로를 계획.\n"
    "list_obstacles 도구로 현재 장애물 목록을 실시간 확인 가능.\n"
    "\n"
    "복합 도형 워크플로우:\n"
    "1. calculate_rectangle_bounds로 각 컴포넌트 좌표 계산\n"
    "2. check_rectangles_overlap으로 겹침 확인 (겹치면 안 되는 것만)\n"
    "3. draw_rectangle, draw_line_segment, draw_polyline 순차 실행\n"
    "\n"
    "리셋 후에는 추가 명령 보내지 마. set_pen에서 off=0이 펜 켜기, off=1이 끄기야."
)


def log(msg, style="dim cyan"):
    """콘솔에 디버그 로그 출력."""
    Console().print(f"  [{style}][LOG] {msg}[/{style}]")


# 전처리 레이어: 키워드 → 도구 힌트 매핑
QUERY_HINTS = [
    # get_turtle_pose
    (["위치", "어디", "포즈", "pose", "좌표"], "get_turtle_pose를 호출해라"),
    # teleport_absolute
    (["이동", "옮겨", "텔레포트", "가줘", "보내줘"], "teleport_absolute를 사용해라"),
    # teleport_relative
    (["왼쪽", "좌회전"], "teleport_relative(name='turtle1', linear=0, angular=1.57)을 호출해라"),
    (["오른쪽", "우회전"], "teleport_relative(name='turtle1', linear=0, angular=-1.57)을 호출해라"),
    # publish_twist_to_cmd_vel
    (["앞으로", "전진"], "publish_twist_to_cmd_vel(name='turtle1', velocity=2.0, lateral=0, angle=0, steps=1)을 호출해라"),
    (["뒤로", "후진"], "publish_twist_to_cmd_vel(name='turtle1', velocity=-2.0, lateral=0, angle=0, steps=1)을 호출해라"),
    # set_pen
    (["빨간", "빨강"], "set_pen(name='turtle1', r=255, g=0, b=0, width=2, off=0)을 호출해라"),
    (["파란", "파랑"], "set_pen(name='turtle1', r=0, g=0, b=255, width=2, off=0)을 호출해라"),
    (["초록", "녹색"], "set_pen(name='turtle1', r=0, g=255, b=0, width=2, off=0)을 호출해라"),
    (["노란", "노랑"], "set_pen(name='turtle1', r=255, g=255, b=0, width=2, off=0)을 호출해라"),
    (["하얀", "하양", "흰색"], "set_pen(name='turtle1', r=255, g=255, b=255, width=2, off=0)을 호출해라"),
    (["펜 끄", "선 안", "안그"], "set_pen(name='turtle1', r=0, g=0, b=0, width=2, off=1)을 호출해라"),
    (["펜 켜", "선 그"], "set_pen(name='turtle1', r=0, g=0, b=0, width=2, off=0)을 호출해라"),
    (["굵게", "두껍"], "set_pen에서 width=5로 설정해라"),
    (["가늘", "얇"], "set_pen에서 width=1로 설정해라"),
    # spawn_turtle
    (["생성", "만들어", "스폰", "spawn", "새 거북"], "spawn_turtle을 사용해라. name, x, y, theta를 지정해라"),
    # kill_turtle
    (["삭제", "없애", "제거", "kill", "죽여"], "kill_turtle을 사용해라. names 리스트로 전달해라"),
    # stop_turtle
    (["멈춰", "정지", "스톱", "stop"], "stop_turtle을 호출해라"),
    # reset_turtlesim
    (["초기화", "리셋", "reset"], "reset_turtlesim을 호출해라"),
    # clear_turtlesim
    (["배경 지우", "화면 지우", "지워", "clear"], "clear_turtlesim을 호출해라"),
    # has_moved_to_expected_coordinates
    (["도착했", "도달했", "확인해", "위치 확인"], "has_moved_to_expected_coordinates로 거북이가 목표 좌표에 도착했는지 확인해라"),
    # draw_line_segment
    (["직선", "라인"], "draw_line_segment를 사용해라"),
    # draw_rectangle
    (["사각형", "네모", "박스", "상자"], "draw_rectangle을 사용해라"),
    # draw_polyline
    (["별"], "draw_polyline을 사용해서 오각형 별을 그려. 꼭짓점 5개를 72도 간격으로 계산하고 0→2→4→1→3 순서로 연결, closed=True"),
    (["삼각형", "세모"], "draw_polyline으로 3개 점을 연결해라, closed=True"),
    (["오각형"], "draw_polyline으로 5개 점을 연결, closed=True"),
    (["육각형"], "draw_polyline으로 6개 점을 연결, closed=True"),
    (["다각형"], "draw_polyline을 사용해라, closed=True"),
    (["집", "하우스"], "draw_rectangle로 벽을 그리고 draw_polyline으로 삼각형 지붕을 그려라"),
    # draw_circle
    (["원", "동그라미", "서클"], "draw_circle을 사용해라"),
    # draw_arc
    (["호", "반원", "아크", "arc", "곡선"], "draw_arc를 사용해라. center_x, center_y, radius, start_angle, arc_angle을 지정해라"),
    # calculate_rectangle_bounds
    (["사각형 좌표", "꼭짓점 계산", "레이아웃", "배치 계산"], "calculate_rectangle_bounds로 사각형의 네 꼭짓점과 중심을 계산해라"),
    # check_rectangles_overlap
    (["겹침", "겹치", "충돌 확인", "overlap"], "check_rectangles_overlap으로 두 사각형이 겹치는지 확인해라"),
    # add_obstacle
    (["장애물 추가", "obstacle add"], "add_obstacle를 사용해라"),
    # remove_obstacle
    (["장애물 삭제", "장애물 제거", "obstacle remove"], "remove_obstacle를 사용해라"),
    # list_obstacles
    (["장애물 목록", "장애물 보여", "obstacle list"], "list_obstacles를 사용해라"),
    # 정적 장애물 지점
    (["a-point", "a포인트", "A지점"], "teleport_absolute(name='turtle1', x=1.0, y=5.0, theta=0)을 호출해라"),
    (["b-point", "b포인트", "B지점"], "teleport_absolute(name='turtle1', x=10.0, y=5.0, theta=0)을 호출해라"),
    (["c-point", "c포인트", "C지점"], "teleport_absolute(name='turtle1', x=6.0, y=7.0, theta=0)을 호출해라"),
    (["wet", "웻", "미끄러운"], "wet 영역은 (5,4)~(7,6)이다. 이 영역을 피해서 이동해라"),
]


def preprocess_query(query: str) -> str:
    """사용자 쿼리에서 키워드를 찾아 도구 힌트를 추가합니다."""
    hints = []
    for keywords, hint in QUERY_HINTS:
        for kw in keywords:
            if kw in query:
                hints.append(hint)
                break
    if hints:
        hint_text = " / ".join(hints)
        processed = f"{query}\n\n[도구 힌트: {hint_text}]"
        log(f"전처리: {len(hints)}개 힌트 매칭 → {hint_text}")
        return processed
    log("전처리: 매칭된 힌트 없음 - 원본 쿼리 전달", "yellow")
    return query


class KoreanTurtleAgent:

    def __init__(
        self,
        streaming: bool = False,
        obstacle_store: Optional[ObstacleStore] = None,
    ):
        self.__streaming = streaming
        self.__chat_history = []

        self._obstacle_store = obstacle_store or ObstacleStore()
        obstacle_tools.configure_obstacle_store(self._obstacle_store)

        self.__llm = get_llm(streaming=streaming).with_config({"streaming": streaming})

        log(f"LLM 로드: {type(self.__llm).__name__}")
        log(f"등록 도구: {len(TOOLS)}개 - {[t.name for t in TOOLS]}")

        prompt = ChatPromptTemplate.from_messages([
            ("system", SYSTEM_PROMPT),
            MessagesPlaceholder(variable_name="chat_history"),
            ("user", "{input}"),
            MessagesPlaceholder(variable_name="agent_scratchpad"),
        ])

        agent = create_tool_calling_agent(
            llm=self.__llm,
            tools=TOOLS,
            prompt=prompt,
        )

        self.__executor = AgentExecutor(
            agent=agent,
            tools=TOOLS,
            stream_runnable=streaming,
            verbose=False,
            max_iterations=30,
            handle_parsing_errors=True,
        )

        self._check_ros_status()

        self.examples = [
            "오각형 별 그려줘",
            "원 그려줘",
            "사각형 그려줘",
            "삼각형 그려줘",
            "빨간색 펜으로 바꿔줘",
            "거북이 위치 알려줘",
            "초기화해줘",
            "장애물 목록 보여줘",
            "집 그려줘",
        ]

    def _check_ros_status(self):
        """ROS와 turtlesim 연결 상태를 확인합니다."""
        try:
            topics = rospy.get_published_topics()
            topic_names = [t[0] for t in topics]
            has_turtlesim = any("turtle1" in t for t in topic_names)
            log(f"ROS 연결: OK, 토픽 {len(topics)}개")
            if has_turtlesim:
                log("turtlesim: 연결됨 (turtle1 발견)")
            else:
                log("turtlesim: 없음! turtle1 토픽이 없습니다", "bold red")
        except Exception as e:
            log(f"ROS 연결 실패: {e}", "bold red")

    def invoke(self, query: str) -> str:
        log(f"입력: '{query}'")
        log(f"대화 히스토리: {len(self.__chat_history)}개 메시지")
        processed_query = preprocess_query(query)
        start_time = time.time()

        try:
            result = self.__executor.invoke(
                {"input": processed_query, "chat_history": self.__chat_history}
            )
            elapsed = time.time() - start_time
            log(f"완료: {elapsed:.1f}초 소요")

            steps = result.get("intermediate_steps", [])
            if steps:
                log(f"도구 호출 {len(steps)}회")
            else:
                log("도구 호출 없음 - LLM이 텍스트로만 응답", "yellow")

            self.__chat_history.extend([
                HumanMessage(content=query),
                AIMessage(content=result["output"]),
            ])
            return result["output"]
        except KeyboardInterrupt:
            raise
        except Exception as e:
            elapsed = time.time() - start_time
            log(f"오류 발생 ({elapsed:.1f}초): {type(e).__name__}: {e}", "bold red")
            return f"오류 발생: {str(e)}"

    async def astream(self, query: str):
        log(f"입력: '{query}'")
        log(f"대화 히스토리: {len(self.__chat_history)}개 메시지")
        processed_query = preprocess_query(query)
        start_time = time.time()
        tool_count = 0

        try:
            final_output = ""
            async for event in self.__executor.astream_events(
                input={"input": processed_query, "chat_history": self.__chat_history},
                config={"run_name": "Agent"},
                version="v2",
            ):
                kind = event["event"]
                if kind == "on_chat_model_stream":
                    content = event["data"]["chunk"].content
                    if content:
                        final_output += content
                        yield {"type": "token", "content": content}
                elif kind == "on_tool_start":
                    tool_count += 1
                    tool_input = event["data"].get("input", "")
                    log(f"도구 시작: {event['name']}({tool_input})")
                    yield {"type": "tool_start", "name": event["name"]}
                elif kind == "on_tool_end":
                    output = str(event["data"].get("output", ""))
                    short = output[:100] + "..." if len(output) > 100 else output
                    log(f"도구 완료: {event['name']} → {short}")
                    yield {"type": "tool_end", "name": event["name"]}
                elif kind == "on_chain_end":
                    if event["name"] == "Agent":
                        chain_output = event["data"].get("output", {}).get("output")
                        if chain_output:
                            final_output = chain_output
                            yield {"type": "final", "content": chain_output}

            elapsed = time.time() - start_time
            if tool_count == 0:
                log(f"도구 호출 없음 - LLM이 텍스트로만 응답 ({elapsed:.1f}초)", "yellow")
            else:
                log(f"완료: 도구 {tool_count}회 호출, {elapsed:.1f}초 소요")

            if final_output:
                self.__chat_history.extend([
                    HumanMessage(content=query),
                    AIMessage(content=final_output),
                ])
        except KeyboardInterrupt:
            yield {"type": "error", "content": "중단됨"}
        except Exception as e:
            elapsed = time.time() - start_time
            log(f"스트림 오류 ({elapsed:.1f}초): {type(e).__name__}: {e}", "bold red")
            yield {"type": "error", "content": f"오류: {e}"}

    def clear_chat(self):
        count = len(self.__chat_history)
        self.__chat_history = []
        log(f"대화 초기화: {count}개 메시지 삭제")

    async def run(self):
        console = Console()
        os.system("clear")

        greeting = Text("\n거북이 에이전트에 오신 것을 환영합니다!\n")
        greeting.stylize("frame bold blue")
        greeting.append("한국어로 명령하세요. 'exit'=종료, 'clear'=대화초기화\n", style="italic")
        console.print(greeting)

        console.print(Panel(
            "\n".join(f"  {i+1}. {ex}" for i, ex in enumerate(self.examples)),
            title="명령 예시",
            border_style="cyan",
        ))

        while True:
            try:
                console.print()
                user_input = input("[명령] > ").strip()

                if not user_input:
                    continue
                if user_input.lower() == "exit":
                    console.print("[bold green]종료합니다.[/bold green]")
                    break
                if user_input.lower() == "clear":
                    self.clear_chat()
                    os.system("clear")
                    console.print("[green]대화 초기화 완료[/green]")
                    continue

                if self.__streaming:
                    await self._stream_response(console, user_input)
                else:
                    self._print_response(console, user_input)

            except KeyboardInterrupt:
                console.print("\n[yellow]중단됨. 'exit'로 종료.[/yellow]")
                continue
            except Exception as e:
                console.print(f"[red]오류: {e}[/red]")
                continue

    def _print_response(self, console: Console, query: str):
        response = self.invoke(query)
        console.print(Panel(Markdown(response), title="응답", border_style="green"))

    async def _stream_response(self, console: Console, query: str):
        content = ""
        panel = Panel("", title="응답 중...", border_style="green")
        with Live(panel, console=console, auto_refresh=False) as live:
            async for event in self.astream(query):
                if event["type"] == "token":
                    content += event["content"]
                    panel.renderable = Markdown(content)
                    live.refresh()
                elif event["type"] == "tool_start":
                    console.print(f"  [dim]도구: {event['name']}[/dim]")
                elif event["type"] == "final":
                    content = event["content"]
                    panel.renderable = Markdown(content)
                    panel.title = "응답"
                    live.refresh()


def main(
    obstacle_store: Optional[ObstacleStore] = None,
    *,
    load_static_world_once: bool = True,
) -> None:
    dotenv.load_dotenv(dotenv.find_dotenv())

    streaming = rospy.get_param("~streaming", False)
    if obstacle_store is None:
        obstacle_store = ObstacleStore()
    if load_static_world_once:
        load_static_world(obstacle_store)

    agent = KoreanTurtleAgent(streaming=streaming, obstacle_store=obstacle_store)
    try:
        asyncio.run(agent.run())
    except KeyboardInterrupt:
        print("\n[종료]")
    except Exception as e:
        print(f"\n[오류: {e}]")
        sys.exit(1)


if __name__ == "__main__":
    rospy.init_node("korean_turtle_agent", log_level=rospy.INFO)

    def _ros_spin() -> None:
        rospy.spin()

    spin_thread = threading.Thread(target=_ros_spin, name="rospy_spin", daemon=True)
    spin_thread.start()

    obstacle_store = ObstacleStore()
    load_static_world(obstacle_store)
    pose_hub = PoseHub()
    pose_log_consumer = PoseLogConsumer(
        period=float(rospy.get_param("~pose_log_interval", POSE_LOG_INTERVAL_SEC))
    )
    collision_jsonl = CollisionJsonlWriter(pose_log_consumer.collision_log_path())
    if get_bool_param("~collision_log_to_console", False):
        rospy.loginfo("collision log: %s", collision_jsonl.path)
    collision_monitor = CollisionMonitor(
        obstacle_store,
        turtle_radius=float(rospy.get_param("~turtle_collision_radius", 0.5)),
        emit_stay=get_bool_param("~collision_emit_stay", False),
        event_sink=make_collision_event_sink(collision_jsonl),
    )
    pose_hub.register_consumer(pose_log_consumer.on_pose)
    pose_hub.register_consumer(collision_monitor.on_pose)
    registered_turtles = pose_hub.start_from_ros_graph_once()
    rospy.loginfo("PoseHub registered turtles: %s", ", ".join(registered_turtles))
    turtle_tools.configure_turtle_lifecycle_listener(pose_hub)
    try:
        main(obstacle_store=obstacle_store, load_static_world_once=False)
    finally:
        turtle_tools.configure_turtle_lifecycle_listener(None)
        pose_hub.stop()
        pose_log_consumer.close()
        collision_jsonl.close()
        rospy.signal_shutdown("korean_turtle_agent exiting")
        spin_thread.join(timeout=5.0)
