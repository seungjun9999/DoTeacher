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
    context += """
    아래는 작품 목록과 설명입니다:
    - 난맹첩: 김정희의 난맹첩은 조선 후기의 걸작으로, 난초의 섬세함과 강한 생명력을 유려한 필치로 표현한 수묵화입니다.
    - 아홉번째 파도: 이반 아이바조프스키의 아홉번째 파도는 거대한 파도가 몰아치는 바다를 배경으로 희망과 생존의 이야기를 담은 유화 작품입니다.
    - 부유세계 마타베이의 걸작: 우타가와 구니요시의 이 작품은 전설과 신화를 소재로 한 생동감 넘치는 우키요에 목판화입니다.
    - 겨울 풍경: 카밀 피사로의 겨울 풍경은 차가운 겨울날의 평화로운 장면을 따뜻하고 부드러운 색채로 표현한 인상파 유화입니다.
    - 게르니카: 파블로 피카소의 게르니카는 스페인 내전의 비극을 강렬한 흑백 대비와 왜곡된 인물들로 생생하게 전달하는 반전 유화입니다.
    - 무제: 잭슨 폴록의 무제는 드립 페인팅 기법을 사용한 추상 표현주의의 대표작으로, 강렬한 에너지를 전달합니다.
    - 송하음다도: 심사정의 송하음다도는 소나무 아래에서 차를 마시며 시를 읊는 모습을 섬세한 필치와 깊이 있는 구도로 표현한 수묵화입니다.
    - 기억의 지속: 살바도르 달리의 기억의 지속은 녹아내리는 시계를 통해 시간의 유동성과 무의미함을 표현한 초현실주의 유화입니다.
    - 밤의 카페 테라스: 빈센트 반 고흐의 밤의 카페 테라스는 밝은 노란색과 어두운 파란색의 대조로 밤의 활기찬 분위기를 표현한 작품입니다.
    - 별이 빛나는 밤: 반 고흐의 별이 빛나는 밤은 소용돌이치는 구름과 별들이 강렬한 색채로 묘사된 감정과 자연이 융합된 독특한 유화입니다.
    - 서당: 김홍도의 서당은 조선 시대 서당에서의 교육 장면을 생생하게 그린 수묵화로, 학생들의 다양한 표정과 자세가 사실적으로 묘사되었습니다.
    - 파적도: 김득신의 파적도는 밤에 고양이가 쥐를 쫓는 모습을 섬세하게 포착하여 표현한 동양 고전 수묵화입니다.
    - 절규: 에드바르 뭉크의 절규는 공포와 불안, 절망감을 강렬하게 표현한 서양 표현주의 유화입니다.
    - 단오 풍정: 신윤복의 단오 풍정은 조선 시대 단오절의 생동감 있는 풍경을 그린 수묵화로, 물가에서 노는 사람들과 그네 타는 여인들이 묘사되어 있습니다.
    - 아테네 학당: 라파엘로의 아테네 학당은 고대 그리스의 철학자들과 학자들을 그린 르네상스 시대의 걸작 프레스코입니다.
    - 영묘도 대련: 오원 장승업의 영묘도 대련은 자연 속에서 동물들의 생동감을 담은 조선 시대의 동양 고전 수묵화입니다.
    - 인왕제색도: 정선의 인왕제색도는 비가 온 후의 인왕산을 맑은 공기와 산의 청명함이 느껴지도록 그린 대표적인 수묵화입니다.
    - 가시 목걸이 자화상: 프리다 칼로의 가시 목걸이 자화상은 그녀의 고통과 열정을 강렬한 색채와 상징적인 요소로 담아낸 초현실주의 유화입니다.
    - 미산이곡: 장승업의 미산이곡은 아름다운 산과 기이한 계곡을 생생하게 담아낸 조선 시대의 산수화입니다.
    - 진주 귀걸이를 한 소녀: 요하네스 페르메이르의 진주 귀걸이를 한 소녀는 소녀의 맑은 눈동자와 빛나는 진주 귀걸이가 인상적인 섬세한 초상화입니다.
    
    또한, 전시회에서 화장실과 엘리베이터의 위치는 다음과 같습니다:
    - 화장실: 2층 엘리베이터 앞
    - 엘리베이터: 입구 오른쪽
    - 입구 : 1층 엘리베이터 왼쪽
    - 출구 : 입구 왼쪽
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
