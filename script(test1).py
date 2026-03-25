from flask import Flask, request
import os

app = Flask(__name__)

SAVE_DIR = 'cat_captures'
os.makedirs(SAVE_DIR, exist_ok=True)


@app.route('/upload', methods=['POST'])
def upload_image():
    # Grab the raw binary data sent by the ESP32
    image_data = request.data

    # Check if the ESP32 sent an empty payload
    if not image_data:
        return "No image data received", 400

    # Save the raw bytes directly to a .jpg file
    save_path = os.path.join(SAVE_DIR, 'latest_capture.jpg')
    with open(save_path, 'wb') as f:
        f.write(image_data)

    full_path = os.path.abspath(save_path)
    print(f"Success! Saved new cat photo to: {full_path}")

    return "Image successfully received by server", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)