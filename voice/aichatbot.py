# -*- coding: utf-8 -*-

from dotenv import load_dotenv
import os
import speech_recognition as sr
from google.cloud import texttospeech
from openai import OpenAI
import pygame
import time
from datetime import datetime
import glob
import threading

load_dotenv()
openai_api_key = os.environ.get('API_KEY')

# OpenAI API 키 설정
client = OpenAI(api_key=openai_api_key)

# Google Cloud 서비스 계정 키 파일 경로 설정
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "C:\\Users\\SSAFY\\Desktop\\LLM\\AIchatbot\\llm.json"

# 음성 인식 객체 생성
recognizer = sr.Recognizer()

# Google Cloud 클라이언트 생성
tts_client = texttospeech.TextToSpeechClient()

# 오디오 파일을 저장할 디렉토리 설정
audio_dir = "audio_files"
os.makedirs(audio_dir, exist_ok=True)

# 멈춤 플래그 설정
stop_playback = threading.Event()

# 작품 목록 및 설명 데이터
exhibition_info = {
    "name": "세계 명화 전시회",
    "dates": "2024년 8월 5일부터 8월 30일까지",
    "place": {
        "화장실": "2층 엘리베이터 앞",
        "엘리베이터": "입구 오른쪽",
        "화장실2": "입구 왼쪽"
    },
    "artworks": {
        "난맹첩": {"description": "김정희의 난맹첩은 조선 후기의 걸작으로, 난초의 섬세함과 강한 생명력을 유려한 필치로 표현한 수묵화입니다.", "location": "1번 전시실"},
        "아홉번째 파도": {"description": "이반 아이바조프스키의 아홉번째 파도는 거대한 파도가 몰아치는 바다를 배경으로 희망과 생존의 이야기를 담은 유화 작품입니다.", "location": "2번 전시실"},
        "부유세계 마타베이의 걸작": {"description": "우타가와 구니요시의 이 작품은 전설과 신화를 소재로 한 생동감 넘치는 우키요에 목판화입니다.", "location": "3번 전시실"},
        "겨울 풍경": {"description": "카밀 피사로의 겨울 풍경은 차가운 겨울날의 평화로운 장면을 따뜻하고 부드러운 색채로 표현한 인상파 유화입니다.", "location": "4번 전시실"},
        "게르니카": {"description": "파블로 피카소의 게르니카는 스페인 내전의 비극을 강렬한 흑백 대비와 왜곡된 인물들로 생생하게 전달하는 반전 유화입니다.", "location": "5번 전시실"},
        "무제(1948)": {"description": "잭슨 폴록의 무제(1948)는 드립 페인팅 기법을 사용한 추상 표현주의의 대표작으로, 강렬한 에너지를 전달합니다.", "location": "6번 전시실"},
        "송하음다도": {"description": "심사정의 송하음다도는 소나무 아래에서 차를 마시며 시를 읊는 모습을 섬세한 필치와 깊이 있는 구도로 표현한 수묵화입니다.", "location": "7번 전시실"},
        "시계의 연속성": {"description": "살바도르 달리의 시계의 연속성은 녹아내리는 시계를 통해 시간의 유동성과 무의미함을 표현한 초현실주의 유화입니다.", "location": "8번 전시실"},
        "밤의 카페 테라스": {"description": "빈센트 반 고흐의 밤의 카페 테라스는 밝은 노란색과 어두운 파란색의 대조로 밤의 활기찬 분위기를 표현한 작품입니다.", "location": "9번 전시실"},
        "별이 빛나는 밤": {"description": "반 고흐의 별이 빛나는 밤은 소용돌이치는 구름과 별들이 강렬한 색채로 묘사된 감정과 자연이 융합된 독특한 유화입니다.", "location": "10번 전시실"},
        "서당": {"description": "김홍도의 서당은 조선 시대 서당에서의 교육 장면을 생생하게 그린 수묵화로, 학생들의 다양한 표정과 자세가 사실적으로 묘사되었습니다.", "location": "11번 전시실"},
        "야묘도추": {"description": "김득신의 야묘도추는 밤에 고양이가 쥐를 쫓는 모습을 섬세하게 포착하여 표현한 동양 고전 수묵화입니다.", "location": "12번 전시실"},
        "절규": {"description": "에드바르 뭉크의 절규는 공포와 불안, 절망감을 강렬하게 표현한 서양 표현주의 유화입니다.", "location": "13번 전시실"},
        "단오 풍정": {"description": "신윤복의 단오 풍정은 조선 시대 단오절의 생동감 있는 풍경을 그린 수묵화로, 물가에서 노는 사람들과 그네 타는 여인들이 묘사되어 있습니다.", "location": "14번 전시실"},
        "아테네 학당": {"description": "라파엘로의 아테네 학당은 고대 그리스의 철학자들과 학자들을 그린 르네상스 시대의 걸작 프레스코입니다.", "location": "15번 전시실"},
        "영묘도 대련": {"description": "오원 장승업의 영묘도 대련은 자연 속에서 동물들의 생동감을 담은 조선 시대의 동양 고전 수묵화입니다.", "location": "16번 전시실"},
        "인왕제색도": {"description": "정선의 인왕제색도는 비가 온 후의 인왕산을 맑은 공기와 산의 청명함이 느껴지도록 그린 대표적인 수묵화입니다.", "location": "17번 전시실"},
        "가시 목걸이 자화상": {"description": "프리다 칼로의 가시 목걸이 자화상은 그녀의 고통과 열정을 강렬한 색채와 상징적인 요소로 담아낸 초현실주의 유화입니다.", "location": "18번 전시실"},
        "미산이곡": {"description": "장승업의 미산이곡은 아름다운 산과 기이한 계곡을 생생하게 담아낸 조선 시대의 산수화입니다.", "location": "19번 전시실"},
        "진주 귀걸이를 한 소녀": {"description": "요하네스 페르메이르의 진주 귀걸이를 한 소녀는 소녀의 맑은 눈동자와 빛나는 진주 귀걸이가 인상적인 섬세한 초상화입니다.", "location": "20번 전시실"}
    }
}

def recognize_speech_from_mic():
    with sr.Microphone() as source:
        print("소음 수준을 조정하는 중입니다. 잠시만 기다려 주세요...")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        print("말하세요...")

        try:
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
            return audio
        except sr.WaitTimeoutError:
            print("입력 시간 초과입니다.")
            return None

def recognize_google_speech(audio):
    try:
        text = recognizer.recognize_google(audio, language='ko-KR')
        print(f"인식된 텍스트: {text}")
        return text
    except sr.UnknownValueError:
        print("도선생이 당신의 말을 이해하지 못했습니다.")
        return ""
    except sr.RequestError as e:
        print(f"도선생 서비스에 문제가 발생했습니다; {e}")
        return ""

def get_response_from_openai(prompt, context):
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": context},
            {"role": "user", "content": prompt},
        ],
        max_tokens=200,
    )
    return response.choices[0].message.content.strip()

def synthesize_text(text, output_audio_file):
    synthesis_input = texttospeech.SynthesisInput(text=text)
    voice = texttospeech.VoiceSelectionParams(
        language_code="ko-KR",
        ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )
    response = tts_client.synthesize_speech(
        input=synthesis_input, voice=voice, audio_config=audio_config
    )
    with open(output_audio_file, "wb") as out:
        out.write(response.audio_content)
    print(f"Audio content written to file {output_audio_file}")

def play_audio(file):
    pygame.mixer.init()
    pygame.mixer.music.load(file)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        if stop_playback.is_set():
            pygame.mixer.music.stop()
            break
        time.sleep(0.1)

def manage_audio_files(directory, max_files=10):
    files = glob.glob(os.path.join(directory, "*.mp3"))
    if len(files) > max_files:
        files.sort(key=os.path.getctime)  # 생성 시간 기준으로 정렬
        while len(files) > max_files:
            os.remove(files.pop(0))  # 가장 오래된 파일 삭제
            print(f"Removed old audio file: {files[0]}")

def recommend_related_artworks(selected_artworks):
    related_artworks = []
    for artwork in selected_artworks:
        related_artworks.append(artwork)
        related_artworks.extend([other for other in exhibition_info["artworks"] if other != artwork])
    return [str(list(exhibition_info["artworks"].keys()).index(artwork) + 1) for artwork in related_artworks]

def handle_user_input(user_input, context):
    # Check if user input matches any artwork description request
    for artwork, details in exhibition_info["artworks"].items():
        if artwork in user_input and any(keyword in user_input for keyword in ["설명해줘", "알려줘", "설명"]):
            return details["description"]
    
    # 기본 프롬프트를 사용하여 질문을 처리
    response = get_response_from_openai(user_input, context)
    
    if "전시회" not in response and any(keyword not in response for keyword in exhibition_info["artworks"].keys()):
        return "전시회 관련 말이 아닌 것 같습니다. 다시 말해주세요."
    return response

def conversation_thread(prompt, context):
    response_text = handle_user_input(prompt, context)
    print(f"Do-Saeng: {response_text}")
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    output_audio_file = os.path.join(audio_dir, f"response_{timestamp}.mp3")
    synthesize_text(response_text, output_audio_file)
    play_audio(output_audio_file)
    manage_audio_files(audio_dir)

def start_conversation(context):
    audio = recognize_speech_from_mic()
    if audio is not None:
        recognized_text = recognize_google_speech(audio)
        if recognized_text:
            print(f"User: {recognized_text}")
            if "그만" in recognized_text:
                stop_playback.set()
                print("Playback stopped.")
            else:
                stop_playback.clear()
                if "너는 누구니" in recognized_text:
                    prompt = "안녕하세요, 저는 전시회 안내 도우미 도선생입니다. 저는 전시회 정보를 제공하고 안내하는 역할을 합니다. 궁금한 점이 있으시면 언제든지 물어보세요!"
                else:
                    prompt = recognized_text
                threading.Thread(target=conversation_thread, args=(prompt, context)).start()

def main():
    context = """
    You are 도선생, a friendly and helpful robot who introduces and provides information about '세계 명화 전시회'.
    The exhibition runs from 2024년 8월 5일부터 8월 30일까지.
    You can only provide information about the exhibition and its artworks listed below.
    Here is a list of the artworks and their descriptions:
    """

    for i, (artwork, details) in enumerate(exhibition_info["artworks"].items(), 1):
        context += f"\n{i}. {artwork}: {details['description']} 위치는 {details['location']}"

    context += """
    Additionally, here is the information about locations in the exhibition:
    화장실: 2층 엘리베이터 앞
    엘리베이터: 입구 오른쪽
    화장실2: 입구 왼쪽
    """

    while True:
        start_conversation(context)

if __name__ == "__main__":
    main()
