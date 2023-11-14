import google.cloud.texttospeech as tts

class TtsApi:

    #Changed function from https://codelabs.developers.google.com/codelabs/cloud-text-speech-python3#5
    #Function sends text to API and save it as a .wav file
    def text_to_wav(self, voice_name: str, text: str):
        language_code = "-".join(voice_name.split("-")[:2])
        text_input = tts.SynthesisInput(text=text)
        voice_params = tts.VoiceSelectionParams(
            language_code=language_code, name=voice_name
        )
        audio_config = tts.AudioConfig(audio_encoding=tts.AudioEncoding.LINEAR16)

        client = tts.TextToSpeechClient()
        response = client.synthesize_speech(
            input=text_input,
            voice=voice_params,
            audio_config=audio_config,
        )

        filename = f"audio_files/{voice_name}.wav"
        with open(filename, "wb") as out:
            out.write(response.audio_content)
            #print(f'Generated speech saved to "{filename}"')