# -*- coding: utf-8 -*-

from dotenv import load_dotenv
import os
import paho.mqtt.client as mqtt
from google.cloud import speech_v1p1beta1 as speech
from google.cloud import texttospeech_v1 as texttospeech
from openai import OpenAI
import pyaudio
import pygame
import time

# 환경 변수 로드 및 클라이언트 초기화
def initialize_clients():
    load_dotenv()
    openai_api_key = os.environ.get('API_KEY')
    return {
        "openai_client": OpenAI(api_key=openai_api_key),
        "speech_client": speech.SpeechClient(),
        "tts_client": texttospeech.TextToSpeechClient(),
    }

# MQTT 설정 및 초기화
def initialize_mqtt():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(BROKER, PORT, 60)
    mqtt_client.loop_start()  # 비동기 MQTT 루프 시작
    return mqtt_client

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(FROM_JETSON_TOPIC)

def on_message(client, userdata, msg):
    print(f"Received message: {msg.topic} {msg.payload.decode()}")

    dec_msg = msg.payload.decode()
    if dec_msg in ["ment_start", "ment_end", "beep", "p_sound"]:
        if dec_msg == "ment_start":
            play_audio('ment_start.mp3')
        elif dec_msg == "ment_end":
            play_audio('ment_end.mp3')
        elif dec_msg == "beep":
            play_audio('beep.mp3')
        elif dec_msg =="p_sound":
            play_audio('5s_camera.mp3')
    else:
        process_payload(msg.payload.decode())
            

def process_payload(payload):
    try:
        if payload.startswith("desc:"):
            handle_desc(int(payload.split(":")[1]))
        elif payload.startswith("next:"):
            handle_next(int(payload.split(":")[1]))
        else:
            print("Unknown message format.")
    except ValueError:
        print(f"Failed to parse signal from payload: {payload}")

def recognize_speech_from_mic(timeout=5.0):
    """마이크로부터 실시간 입력을 받아 음성을 인식합니다."""
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 48000
    THRESHOLD = 500

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    print("Recording...")
    frames, silent_chunks = [], 0
    max_silent_chunks = int(RATE / CHUNK * timeout)
    
    while silent_chunks <= max_silent_chunks:
        data = stream.read(CHUNK, exception_on_overflow=False)
        frames.append(data)
        silent_chunks = silent_chunks + 1 if max(data) < THRESHOLD else 0
    
    print("Recording finished.")
    stream.stop_stream()
    stream.close()
    p.terminate()

    return process_speech(frames, RATE)

def process_speech(frames, rate):
    audio_content = b''.join(frames)
    audio = speech.RecognitionAudio(content=audio_content)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=rate,
        language_code="ko-KR",
    )

    try:
        response = clients["speech_client"].recognize(config=config, audio=audio)
        return "".join(result.alternatives[0].transcript for result in response.results)
    except Exception as e:
        print(f"Speech recognition failed: {e}")
        return ""

def get_response_from_openai(prompt, context):
    try:
        response = clients["openai_client"].chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": context},
                {"role": "user", "content": prompt},
            ],
            max_tokens=300,
        )
        return response.choices[0].message.content.strip()
    except Exception as e:
        print(f"OpenAI API request failed: {e}")
        return "죄송합니다. 요청을 처리할 수 없습니다."

def handle_user_input(user_input):
    global context

    for keyword in KEYWORDS:
        if keyword in user_input:
            return process_destination_request(user_input)

    context += f"\nUser: {user_input}"
    response = get_response_from_openai(user_input, context)
    context += f"\nDo-Saeng: {response}"
    return response

def process_destination_request(user_input):
    for destination, index in VALID_DESTINATIONS.items():
        if destination in user_input:
            mqtt_client.publish(NAV_JETSON_TOPIC, f"nav:{index}")
            print(f"Sent signal to Jetson Nano: nav:{index}")
            return f"{destination}로 이동 신호를 보냈습니다."
    return "지정된 작품이나 위치가 아닙니다. 다시 말씀해 주세요."

def synthesize_speech(text, output_audio_file_path):
    """텍스트를 음성으로 변환하여 파일로 저장합니다."""
    try:
        input_text = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR",
            name="ko-KR-Wavenet-B",
            ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL,
        )
        audio_config = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.MP3)
        response = clients["tts_client"].synthesize_speech(input=input_text, voice=voice, audio_config=audio_config)

        with open(output_audio_file_path, "wb") as out:
            out.write(response.audio_content)
    except Exception as e:
        print(f"Speech synthesis failed: {e}")

def play_audio(file_path):
    """오디오 파일을 재생합니다."""
    try:
        # Pygame 초기화
        # pygame.mixer.init(devicename="hw:3,0")  # 내장 헤드폰 장치 사용
        pygame.mixer.init(frequency=16000, size=-16, channels=2, buffer=4096)
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        pygame.mixer.music.stop()
    except Exception as e:
        print(f"Audio playback failed: {e}")

def handle_desc(signal):
    global VALID_DESTINATIONS
    if 0 <= signal < len(VALID_DESTINATIONS):
        user_input = f"{list(VALID_DESTINATIONS.keys())[signal]} 설명해줘"
        response = handle_user_input(user_input)
        print(f"도선생: {response}")
        synthesize_speech(response, "response.mp3")
        play_audio("response.mp3")
    else:
        print("유효하지 않은 신호입니다.")

def handle_next(signal):
    global VALID_DESTINATIONS
    if 0 <= signal < len(VALID_DESTINATIONS):
        synthesize_speech(f"{list(VALID_DESTINATIONS.keys())[signal]}으로 안내해드리겠습니다.", "response.mp3")
        play_audio("response.mp3")
    else:
        print("유효하지 않은 신호입니다.")

# 전역 변수들
BROKER = "broker.hivemq.com"
PORT = 1883
FROM_JETSON_TOPIC = "jetson/from"
TO_JETSON_TOPIC = "jetson/to"
NAV_JETSON_TOPIC = "jetson/nav"

VALID_DESTINATIONS = {
    "난맹첩": 0, "아홉번째 파도": 1, "부유세계 마타베이의 걸작": 2, "겨울 풍경": 3, "게르니카": 4,
    "무제": 5, "송하음다도": 6, "시계의 연속성": 7, "밤의 카페 테라스": 8, "별이 빛나는 밤": 9,
    "서당": 10, "야묘도추": 11, "절규": 12, "단오 풍정": 13, "아테네 학당": 14,
    "영묘도 대련": 15, "인왕제색도": 16, "가시 목걸이 자화상": 17, "미산이곡": 18, "진주 귀걸이를 한 소녀": 19,
    "화장실": 20, "엘리베이터": 21, "입구": 22, "출구": 23}

KEYWORDS = ["안 내해", "안내해", "안내 해",
            "가자", "가 자", "가줘", "가 줘", "갈래", "갈 래"
            "데려다줘", "데려다 줘",
            "가고 싶어", "가고싶어",
            "보고싶어", "보고 싶어"]

def main():
    global context
    global mqtt_client
    global clients
    clients = initialize_clients()
    mqtt_client = initialize_mqtt()

    context = """
        당신은 도선생입니다. 당신의 역할은 '세계 명화 전시회'에서 방문객들에게 작품을 친절하고 이해하기 쉽게 설명하는 것입니다. 설명은 20초 정도의 길이로, 한글로 작성해 주세요. 

        응답에는 텍스트 강조 효과나 특수문자를 사용하지 마시고, 자연스럽고 따뜻한 어조로 안내해 주세요. TTS(Text-to-Speech)로 사용될 것이므로, 문장이 너무 길거나 복잡하지 않도록 주의해 주세요. ...
    """
    
    no_input_counter, conversation_active = 0, False

    while True:
        print("마이크로 질문을 하세요")
        user_input = recognize_speech_from_mic(timeout=5.0)

        if user_input:
            print(f"Recognized speech: {user_input}")
            if ("도선생" in user_input or "선생" in user_input or "도선" in user_input or "또선생" in user_input or "더선생" in user_input or conversation_active):
                no_input_counter = 0
                conversation_active = True
                response = handle_user_input(user_input)
                print(f"도선생: {response}")

                synthesize_speech(response, "response.mp3")
                play_audio("response.mp3")
            else:
                print("도선생이 포함되지 않았습니다. 대화를 시작하려면 '도선생'이라고 말해주세요.")
        else:
            no_input_counter += 1
            if no_input_counter >= 3:
                conversation_active = False
                print("입력이 없어 대화가 종료되었습니다. '도선생'이라고 말해 대화를 다시 시작하세요.")

if __name__ == "__main__":
    main()
