import requests
import os

def upload_to_catbox(file_path):
    # 1. 파일 확인
    if not os.path.exists(file_path):
        print(f"[ERROR] '{file_path}' 파일이 없습니다.")
        return

    print(f"[*] '{file_path}' 업로드 중 (Catbox.moe)...")

    try:
        # Catbox API 설정
        url = "https://catbox.moe/user/api.php"
        payload = {'reqtype': 'fileupload'}
        
        # 2. 파일 전송
        with open(file_path, 'rb') as f:
            files = {'fileToUpload': f}
            # 브라우저인 척하기 위한 헤더 추가
            headers = {
                'User-Agent': 'Mozilla/5.0'
            }
            response = requests.post(url, data=payload, files=files, headers=headers)

        # 3. 결과 출력
        if response.status_code == 200:
            link = response.text.strip()
            if link.startswith("https://files.catbox.moe/"):
                print("\n" + "="*60)
                print("[SUCCESS] 업로드 성공!")
                print(f"🔗 다운로드 URL: {link}")
                print("="*60)
                print("위 링크를 복사해서 브라우저에서 다운로드하세요.")
            else:
                print(f"[ERROR] 서버 응답이 올바르지 않습니다: {link}")
        else:
            print(f"[ERROR] HTTP 상태 코드: {response.status_code}")
            print(f"응답 내용: {response.text}")

    except Exception as e:
        print(f"[ERROR] 업로드 중 예외 발생: {e}")

if __name__ == "__main__":
    upload_to_catbox("senior_firmware.bin")