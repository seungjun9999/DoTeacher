# -*- coding: utf-8 -*-

from dotenv import load_dotenv
import os
from openai import OpenAI

load_dotenv()
openai_api_key = os.environ.get('API_KEY')

# OpenAI API 키 설정
client = OpenAI(api_key=openai_api_key)

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
        "난맹첩": "김정희의 난맹첩은 조선 후기의 걸작으로, 난초의 섬세함과 강한 생명력을 유려한 필치로 표현한 수묵화입니다.",
        "아홉번째 파도": "이반 아이바조프스키의 아홉번째 파도는 거대한 파도가 몰아치는 바다를 배경으로 희망과 생존의 이야기를 담은 유화 작품입니다.",
        "부유세계 마타베이의 걸작": "우타가와 구니요시의 이 작품은 전설과 신화를 소재로 한 생동감 넘치는 우키요에 목판화입니다.",
        "겨울 풍경": "카밀 피사로의 겨울 풍경은 차가운 겨울날의 평화로운 장면을 따뜻하고 부드러운 색채로 표현한 인상파 유화입니다.",
        "게르니카": "파블로 피카소의 게르니카는 스페인 내전의 비극을 강렬한 흑백 대비와 왜곡된 인물들로 생생하게 전달하는 반전 유화입니다.",
        "무제(1948)": "잭슨 폴록의 무제(1948)는 드립 페인팅 기법을 사용한 추상 표현주의의 대표작으로, 강렬한 에너지를 전달합니다.",
        "송하음다도": "심사정의 송하음다도는 소나무 아래에서 차를 마시며 시를 읊는 모습을 섬세한 필치와 깊이 있는 구도로 표현한 수묵화입니다.",
        "시계의 연속성": "살바도르 달리의 시계의 연속성은 녹아내리는 시계를 통해 시간의 유동성과 무의미함을 표현한 초현실주의 유화입니다.",
        "밤의 카페 테라스": "빈센트 반 고흐의 밤의 카페 테라스는 밝은 노란색과 어두운 파란색의 대조로 밤의 활기찬 분위기를 표현한 작품입니다.",
        "별이 빛나는 밤": "반 고흐의 별이 빛나는 밤은 소용돌이치는 구름과 별들이 강렬한 색채로 묘사된 감정과 자연이 융합된 독특한 유화입니다.",
        "서당": "김홍도의 서당은 조선 시대 서당에서의 교육 장면을 생생하게 그린 수묵화로, 학생들의 다양한 표정과 자세가 사실적으로 묘사되었습니다.",
        "야묘도추": "김득신의 야묘도추는 밤에 고양이가 쥐를 쫓는 모습을 섬세하게 포착하여 표현한 동양 고전 수묵화입니다.",
        "절규": "에드바르 뭉크의 절규는 공포와 불안, 절망감을 강렬하게 표현한 서양 표현주의 유화입니다.",
        "단오 풍정": "신윤복의 단오 풍정은 조선 시대 단오절의 생동감 있는 풍경을 그린 수묵화로, 물가에서 노는 사람들과 그네 타는 여인들이 묘사되어 있습니다.",
        "아테네 학당": "라파엘로의 아테네 학당은 고대 그리스의 철학자들과 학자들을 그린 르네상스 시대의 걸작 프레스코입니다.",
        "영묘도 대련": "오원 장승업의 영묘도 대련은 자연 속에서 동물들의 생동감을 담은 조선 시대의 동양 고전 수묵화입니다.",
        "인왕제색도": "정선의 인왕제색도는 비가 온 후의 인왕산을 맑은 공기와 산의 청명함이 느껴지도록 그린 대표적인 수묵화입니다.",
        "가시 목걸이 자화상": "프리다 칼로의 가시 목걸이 자화상은 그녀의 고통과 열정을 강렬한 색채와 상징적인 요소로 담아낸 초현실주의 유화입니다.",
        "미산이곡": "장승업의 미산이곡은 아름다운 산과 기이한 계곡을 생생하게 담아낸 조선 시대의 산수화입니다.",
        "진주 귀걸이를 한 소녀": "요하네스 페르메이르의 진주 귀걸이를 한 소녀는 소녀의 맑은 눈동자와 빛나는 진주 귀걸이가 인상적인 섬세한 초상화입니다."
    }
}

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

def handle_user_input(user_input, context):
    # Check if user input matches any artwork description request
    for artwork, description in exhibition_info["artworks"].items():
        if artwork in user_input and any(keyword in user_input for keyword in ["설명해줘", "알려줘", "설명","말해줘","뭐야","알려줘"]):
            return description
    
    # Check if user input matches any place information request
    for place, location in exhibition_info["place"].items():
        if place in user_input and any(keyword in user_input for keyword in ["어디", "위치", "알려줘", "장소"]):
            return f"{place}의 위치는 {location}입니다."
    
    # 기본 프롬프트를 사용하여 질문을 처리
    response = get_response_from_openai(user_input, context)
    
    if "전시회" not in response and all(keyword not in response for keyword in exhibition_info["artworks"].keys()):
        return "전시회 관련 말이 아닌 것 같습니다. 다시 말해주세요."
    return response

def main():
    context = """
    You are 도선생, a friendly and helpful robot who introduces and provides information about '세계 명화 전시회'.
    The exhibition runs from 2024년 8월 5일부터 8월 30일까지.
    You can only provide information about the exhibition and its artworks listed below.
    Here is a list of the artworks and their descriptions:

    1. 난맹첩: 김정희의 난맹첩은 조선 후기의 걸작으로, 난초의 섬세함과 강한 생명력을 유려한 필치로 표현한 수묵화입니다.
    2. 아홉번째 파도: 이반 아이바조프스키의 아홉번째 파도는 거대한 파도가 몰아치는 바다를 배경으로 희망과 생존의 이야기를 담은 유화 작품입니다.
    3. 부유세계 마타베이의 걸작: 우타가와 구니요시의 이 작품은 전설과 신화를 소재로 한 생동감 넘치는 우키요에 목판화입니다.
    4. 겨울 풍경: 카밀 피사로의 겨울 풍경은 차가운 겨울날의 평화로운 장면을 따뜻하고 부드러운 색채로 표현한 인상파 유화입니다.
    5. 게르니카: 파블로 피카소의 게르니카는 스페인 내전의 비극을 강렬한 흑백 대비와 왜곡된 인물들로 생생하게 전달하는 반전 유화입니다.
    6. 무제(1948): 잭슨 폴록의 무제(1948)는 드립 페인팅 기법을 사용한 추상 표현주의의 대표작으로, 강렬한 에너지를 전달합니다.
    7. 송하음다도: 심사정의 송하음다도는 소나무 아래에서 차를 마시며 시를 읊는 모습을 섬세한 필치와 깊이 있는 구도로 표현한 수묵화입니다.
    8. 시계의 연속성: 살바도르 달리의 시계의 연속성은 녹아내리는 시계를 통해 시간의 유동성과 무의미함을 표현한 초현실주의 유화입니다.
    9. 밤의 카페 테라스: 빈센트 반 고흐의 밤의 카페 테라스는 밝은 노란색과 어두운 파란색의 대조로 밤의 활기찬 분위기를 표현한 작품입니다.
    10. 별이 빛나는 밤: 반 고흐의 별이 빛나는 밤은 소용돌이치는 구름과 별들이 강렬한 색채로 묘사된 감정과 자연이 융합된 독특한 유화입니다.
    11. 서당: 김홍도의 서당은 조선 시대 서당에서의 교육 장면을 생생하게 그린 수묵화로, 학생들의 다양한 표정과 자세가 사실적으로 묘사되었습니다.
    12. 야묘도추: 김득신의 야묘도추는 밤에 고양이가 쥐를 쫓는 모습을 섬세하게 포착하여 표현한 동양 고전 수묵화입니다.
    13. 절규: 에드바르 뭉크의 절규는 공포와 불안, 절망감을 강렬하게 표현한 서양 표현주의 유화입니다.
    14. 단오 풍정: 신윤복의 단오 풍정은 조선 시대 단오절의 생동감 있는 풍경을 그린 수묵화로, 물가에서 노는 사람들과 그네 타는 여인들이 묘사되어 있습니다.
    15. 아테네 학당: 라파엘로의 아테네 학당은 고대 그리스의 철학자들과 학자들을 그린 르네상스 시대의 걸작 프레스코입니다.
    16. 영묘도 대련: 오원 장승업의 영묘도 대련은 자연 속에서 동물들의 생동감을 담은 조선 시대의 동양 고전 수묵화입니다.
    17. 인왕제색도: 정선의 인왕제색도는 비가 온 후의 인왕산을 맑은 공기와 산의 청명함이 느껴지도록 그린 대표적인 수묵화입니다.
    18. 가시 목걸이 자화상: 프리다 칼로의 가시 목걸이 자화상은 그녀의 고통과 열정을 강렬한 색채와 상징적인 요소로 담아낸 초현실주의 유화입니다.
    19. 미산이곡: 장승업의 미산이곡은 아름다운 산과 기이한 계곡을 생생하게 담아낸 조선 시대의 산수화입니다.
    20. 진주 귀걸이를 한 소녀: 요하네스 페르메이르의 진주 귀걸이를 한 소녀는 소녀의 맑은 눈동자와 빛나는 진주 귀걸이가 인상적인 섬세한 초상화입니다.
    
    Additionally, here is the information about locations in the exhibition:
    - 화장실: 2층 엘리베이터 앞
    - 엘리베이터: 입구 오른쪽
    - 화장실2: 입구 왼쪽
    """

    while True:
        user_input = input("User: ")
        response = handle_user_input(user_input, context)
        print(f"Do-Saeng: {response}")

if __name__ == "__main__":
    main()
