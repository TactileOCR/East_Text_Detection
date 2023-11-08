

import openai
import os
from dotenv import load_dotenv


# load values from the .env file if it exists
load_dotenv()

# configure OpenAI
openai.api_key = os.getenv("OPENAI_API_KEY")

# Define function to get chatbot response
def get_chatbot_response(prompt):
    response = openai.completions.create(
        model="text-davinci-003",
        prompt=prompt,
        max_tokens=1024,
       
    )
    message = response.choices[0].text.strip()
    return message

# Define function to get chatbot response for a given input string
def get_response(input_string):
    prompt = f"User: {input_string}\nChatbot:"
    return get_chatbot_response(prompt)



response = get_response("Hello, who are you?")
print(response)