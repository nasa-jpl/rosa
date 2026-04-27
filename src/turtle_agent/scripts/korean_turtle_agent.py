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

from math import atan2, sqrt

from langchain.agents import tool as langchain_tool
from tools.turtle import (
    get_turtle_pose,
    teleport_absolute,
    publish_twist_to_cmd_vel,
    set_pen,
    reset_turtlesim,
)
from tools.obstacle import (
    list_obstacles,
)


@langchain_tool
def move_to_point(name: str, x: float, y: float) -> str:
    """목표 좌표 (x, y)까지 직선으로 이동한다. 거리와 각도는 자동 계산된다.
    펜이 켜져 있으면 이동 경로가 그려진다.

    :param name: 거북이 이름 (예: 'turtle1')
    :param x: 목표 x 좌표 (0~11)
    :param y: 목표 y 좌표 (0~11)
    """
    pose = get_turtle_pose.invoke({"names": [name]})
    if "Error" in pose:
        return f"오류: {pose}"
    cx, cy = pose[name].x, pose[name].y
    dx, dy = x - cx, y - cy
    dist = sqrt(dx * dx + dy * dy)
    if dist < 0.1:
        return f"{name}이(가) 이미 ({x}, {y}) 근처에 있습니다."
    angle = atan2(dy, dx)
    teleport_absolute.invoke({
        "name": name, "x": cx, "y": cy, "theta": angle, "hide_pen": True,
    })
    result = publish_twist_to_cmd_vel.invoke({
        "name": name, "velocity": dist, "lateral": 0, "angle": 0, "steps": 1,
    })
    return f"{name}이(가) ({cx:.1f},{cy:.1f})→({x:.1f},{y:.1f}) 이동 완료. {result}"


TOOLS = [
    get_turtle_pose,
    move_to_point,
    teleport_absolute,
    set_pen,
    list_obstacles,
    reset_turtlesim,
]

SYSTEM_PROMPT = (
    "너는 한국어 TurtleSim 에이전트야. 항상 한국어로 대답해.\n"
    "\n"
    "환경: 11x11 2D 공간. 거북이 이름에 /붙이지 마.\n"
    "\n"
    "사용 가능한 도구 6개:\n"
    "- get_turtle_pose: 거북이 현재 위치 확인\n"
    "- move_to_point(name, x, y): 목표 좌표까지 직선 이동 (거리·각도 자동 계산)\n"
    "- teleport_absolute(name, x, y, theta): 순간이동 (선 안 그림)\n"
    "- set_pen(name, r, g, b, width, off): 펜 색상/켜기(off=0)/끄기(off=1)\n"
    "- list_obstacles: 현재 장애물 목록 조회\n"
    "- reset_turtlesim: 환경 초기화\n"
    "\n"
    "정적 장애물 맵:\n"
    "- wet: (5,4)~(7,6) 사각형 — 미끄러운 위험 구간\n"
    "- a-point: (1, 5) 원형 지점\n"
    "- b-point: (10, 5) 원형 지점\n"
    "- c-point: (6, 7) 원형 지점\n"
    "\n"
    "이동 규칙:\n"
    "- 특정 지점으로 이동 → move_to_point 사용\n"
    "- 선을 안 그리고 위치만 옮기기 → teleport_absolute 사용\n"
    "- wet 회피 이동 → wet 영역 (5,4)~(7,6)을 피하는 경유지를 잡아서 move_to_point를 여러 번 호출\n"
    "  예: (10,5)→(7,7)→(4,7)→(1,5) 처럼 위쪽으로 우회\n"
    "- 경로 색 구분 → 이동 전에 set_pen으로 색상 변경\n"
)


def log(msg, style="dim cyan"):
    """콘솔에 디버그 로그 출력."""
    Console().print(f"  [{style}][LOG] {msg}[/{style}]")


# 전처리 레이어: 키워드 → 도구 힌트 매핑
QUERY_HINTS = [
    # move_to_point
    (["a-point", "a포인트", "A지점"], "move_to_point(name='turtle1', x=1.0, y=5.0)을 호출해라"),
    (["b-point", "b포인트", "B지점"], "move_to_point(name='turtle1', x=10.0, y=5.0)을 호출해라"),
    (["c-point", "c포인트", "C지점"], "move_to_point(name='turtle1', x=6.0, y=7.0)을 호출해라"),
    (["이동", "가줘", "직선", "으로 가"], "move_to_point를 사용해라"),
    # teleport_absolute
    (["순간이동", "텔레포트", "옮겨"], "teleport_absolute를 사용해라 (선 안 그림)"),
    # wet 회피
    (["wet", "웻", "미끄러운", "피해", "회피", "우회"],
     "wet 영역 (5,4)~(7,6)을 피해서 이동해라. 경유지를 잡아 move_to_point를 여러 번 호출해라. 예: (7,7)→(4,7) 위쪽 우회"),
    # get_turtle_pose
    (["위치", "어디", "포즈", "���표"], "get_turtle_pose를 호출해라"),
    # set_pen
    (["빨간", "빨강"], "set_pen(name='turtle1', r=255, g=0, b=0, width=2, off=0)을 호출해라"),
    (["파란", "파랑"], "set_pen(name='turtle1', r=0, g=0, b=255, width=2, off=0)을 호출해라"),
    (["초록", "녹색"], "set_pen(name='turtle1', r=0, g=255, b=0, width=2, off=0)을 호출해라"),
    (["펜 끄", "안그"], "set_pen(name='turtle1', r=0, g=0, b=0, width=2, off=1)을 호출해라"),
    (["펜 켜"], "set_pen(name='turtle1', r=0, g=0, b=0, width=2, off=0)을 호출해라"),
    # list_obstacles
    (["장애물", "obstacle"], "list_obstacles를 사용해라"),
    # reset_turtlesim
    (["초기화", "리셋", "reset"], "reset_turtlesim을 호출해라"),
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
            "a-point로 이동해",
            "b-point까지 직선으로 가줘",
            "wet 피해서 a-point로 돌아가",
            "빨간색 펜으로 바꿔줘",
            "거북이 위치 알려줘",
            "장애물 목록 보여줘",
            "초기화해줘",
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
