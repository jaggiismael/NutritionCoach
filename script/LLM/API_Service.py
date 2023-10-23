import openai

class LlmService:
    def __init__(self):
        
        openai.api_base = "https://api.deepinfra.com/v1/openai"
        openai.api_key = ""
        MODEL_DI = "meta-llama/Llama-2-70b-chat-hf"

    def sendPrompt(self, messages, temperature): 
        chat_completion = openai.ChatCompletion.create(
        model="meta-llama/Llama-2-70b-chat-hf",
        messages=messages,
        stream=True,
        temperature=temperature,
        max_token=100
        )

        answer = ""
        for event in chat_completion:
            if 'content' in event.choices[0]['delta']:
                answer += event.choices[0]["delta"]["content"]

        return answer
    

