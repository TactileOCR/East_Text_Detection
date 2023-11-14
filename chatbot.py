from openai import OpenAI
import os
from dotenv import load_dotenv

# Load values from the .env file if it exists
load_dotenv()

# Configure OpenAI
key = os.getenv("OPENAI_API_KEY")

client = OpenAI(api_key=key)

# Define function to get chatbot response
def get_chatbot_response(prompt):
   chat_completion = client.chat.completions.create(
    messages=[
        {
            "role": "user",
            "content" : prompt,
        }
    ],
    model="gpt-3.5-turbo",
)
   print(chat_completion.choices[0].message.content)
   return chat_completion.choices[0].message.content

# Define function to get chatbot response for a given input string
def get_response(input_string):
    return get_chatbot_response(input_string)

def main():
    response = get_response("Hello, who are you?")
    print(response)

main()
