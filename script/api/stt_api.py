from google.cloud import speech

class SttApi:
    #Function from https://cloud.google.com/speech-to-text/docs/sync-recognize
    #Send .wav file to Google Cloud API and receive text
    def transcribe_file(self, speech_file: str) -> speech.RecognizeResponse:
        """Transcribe the given audio file."""
        client = speech.SpeechClient()

        with open(speech_file, "rb") as audio_file:
            content = audio_file.read()

        audio = speech.RecognitionAudio(content=content)
        config = speech.RecognitionConfig(
            language_code="en-US",
            sample_rate_hertz = 44100,
            audio_channel_count = 2,
            enable_automatic_punctuation = True
        )

        response = client.recognize(config=config, audio=audio)

        transcript = ""
        for result in response.results:
            print(f"Transcript: {result.alternatives[0].transcript}")
            transcript = result.alternatives[0].transcript

        return transcript
