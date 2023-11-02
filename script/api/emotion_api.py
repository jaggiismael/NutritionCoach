from google.cloud import language_v2

class EmotionApi:

    #Changed function from https://cloud.google.com/natural-language/docs/analyzing-sentiment
    def getEmotion(self, text):
        client = language_v2.LanguageServiceClient()
        document_type_in_plain_text = language_v2.Document.Type.PLAIN_TEXT

        language_code = "en"
        document = {
            "content": text,
            "type_": document_type_in_plain_text,
            "language_code": language_code,
        }
        encoding_type = language_v2.EncodingType.UTF8

        response = client.analyze_sentiment(
            request={"document": document, "encoding_type": encoding_type}
        )
        # Get overall sentiment of the input document
        print(f"Document sentiment score: {response.document_sentiment.score}")
        print(f"Document sentiment magnitude: {response.document_sentiment.magnitude}")

        return response.document_sentiment.score
