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

import os

import dotenv
from azure.identity import ClientSecretCredential, get_bearer_token_provider
from langchain_openai import AzureChatOpenAI


def get_llm():
    """A helper function to get the LLM instance."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    APIM_SUBSCRIPTION_KEY = os.getenv("APIM_SUBSCRIPTION_KEY")
    default_headers = {}
    if APIM_SUBSCRIPTION_KEY != None:
        # only set this if the APIM API requires a subscription...
        default_headers["Ocp-Apim-Subscription-Key"] = APIM_SUBSCRIPTION_KEY

    # Set up authority and credentials for Azure authentication
    credential = ClientSecretCredential(
        tenant_id=os.getenv("AZURE_TENANT_ID"),
        client_id=os.getenv("AZURE_CLIENT_ID"),
        client_secret=os.getenv("AZURE_CLIENT_SECRET"),
        authority="https://login.microsoftonline.com",
    )

    token_provider = get_bearer_token_provider(
        credential, "https://cognitiveservices.azure.com/.default"
    )

    llm = AzureChatOpenAI(
        azure_deployment=os.getenv("DEPLOYMENT_ID"),
        azure_ad_token_provider=token_provider,
        openai_api_type="azure_ad",
        api_version=os.getenv("API_VERSION"),
        azure_endpoint=os.getenv("API_ENDPOINT"),
        default_headers=default_headers,
    )

    return llm
