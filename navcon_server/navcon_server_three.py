from flask import Flask, request, jsonify
import numpy as np
import cv2
import pickle
from main_nav_lib import *
import os
import argparse



class NavConServer:
    def __init__(self, scene_name ="test", logging_path="logs", url="http://localhost", port=8080):
        print("Starting Server")
        self.app = Flask(__name__)
        self.app.config["MAX_CONTENT_LENGTH"] = 40 * 1024 * 1024  # 20 MB

        # Define the Flask routes
        self.app.add_url_rule("/nav_server", view_func=self.upload, methods=["POST"])
        self.recieved_prompt = None
        self.recieved_image = None

        # Set the logging path
        self.logging_path = logging_path + "/" + scene_name
        os.makedirs(self.logging_path, exist_ok=True)
        self.count = 0

        self.url = url
        self.port = port

    def run(self):
        # Start the Flask app
        print("Server started. Waiting for images.")
        self.app.run(debug=True, host=self.url, port=self.port, use_reloader=False)

    def upload(self):
        # Read image and text data from request data
        img_front_bytes = request.files['image_front'].read()
        img_left_bytes = request.files['image_left'].read()
        img_right_bytes = request.files['image_right'].read()


        text_data = request.form["sentence"]
        self.recieved_prompt = text_data


        self.recieved_image_front = self.bytes_to_image_patch(img_front_bytes, 'image_front')
        self.recieved_image_left = self.bytes_to_image_patch(img_left_bytes, 'image_left')
        self.recieved_image_right = self.bytes_to_image_patch(img_right_bytes, 'image_right')


        result = self.get_nav_command()
        return jsonify(result)

    def get_nav_command(self):
        self.code = get_code(self.recieved_prompt)
        im = [self.recieved_image_left, self.recieved_image_front, self.recieved_image_right]
        self.pickle_data()
        result = execute_code(self.code, im, show_intermediate_steps=False)
        return result

    def bytes_to_image(self, img_bytes, frame):
        img_np = np.frombuffer(img_bytes, dtype=np.uint8)
        img = cv2.imdecode(img_np, flags=cv2.IMREAD_COLOR)
        transform = transforms.Compose([transforms.ToTensor()])
        return transform(img)

    def pickle_data(self):
        name = "nav" + str(self.count)
        file = open(self.logging_path + "/" + name + ".pkl", "wb")
        pickle.dump([self.recieved_prompt, self.code, self.single_image], file)
        file.close()
        self.count = self.count + 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", type=str, default="localhost", help="URL for the server")
    parser.add_argument("--port", type=int, default=8080, help="Port for the server")
    parser.add_argument("--logging_path", type=str, default="logs", help="Path for logging")
    parser.add_argument("--scene_name", type=str, default="scene01", help="Name of the scene")
    args = parser.parse_args()

    server = NavConServer(args.scene_name, args.logging_path, args.url, args.port)
    server.run()

