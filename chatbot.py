
import openai
import os

# Set up OpenAI API credentials
openai.api_key = os.environ["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"]

# Define function to get chatbot response
def get_chatbot_response(prompt):
    response = openai.Completion.create(
        engine="davinci",
        prompt=prompt,
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.5,
    )
    message = response.choices[0].text.strip()
    return message

# Define function to get chatbot response for a given input string
def get_response(input_string):
    prompt = f"User: {input_string}\nChatbot:"
    return get_chatbot_response(prompt)



response = get_response("Hello, who are you?")
print(response)