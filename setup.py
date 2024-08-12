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

import pathlib
from distutils.core import setup

from setuptools import find_packages

here = pathlib.Path(__file__).parent.resolve()
long_description = (here / "README.md").read_text(encoding="utf-8")
rosa_packages = find_packages(where="src")

setup(
    name="jpl-rosa",
    version="1.0.1",
    license="Apache 2.0",
    description="ROSA: the Robot Operating System Agent",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/nasa-jpl/rosa",
    author="Rob Royce",
    author_email="Rob.Royce@jpl.nasa.gov",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Environment :: Console",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Natural Language :: English",
        "Operating System :: Unix",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    keywords="Robotics, Data Science, Machine Learning, Data Engineering, Data Infrastructure, Data Analysis",
    package_dir={"": "src"},
    packages=rosa_packages,
    python_requires=">=3.9, <4",
    install_requires=[
        "PyYAML==6.0.1",
        "python-dotenv>=1.0.1",
        "langchain==0.2.7",
        "langchain-openai==0.1.14",
        "langchain-core==0.2.12",
        "langchain-community",
        "pydantic",
        "pyinputplus",
        "azure-identity",
        "cffi",
        "rich",
        "pillow>=10.4.0",
        "numpy>=1.21.2",
    ],
    project_urls={  # Optional
        "Bug Reports": "https://github.com/nasa-jpl/rosa/issues",
        "Source": "https://github.com/nasa-jpl/rosa",
    },
)
