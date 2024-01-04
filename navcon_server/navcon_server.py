from flask import Flask, request, jsonify
import numpy as np
import cv2
import pickle
from main_nav_lib import *
import os
import argparse



class NavConServer:
    """
    A class representing a navcon server.

    Args:
        scene_name (str): The name of the scene. (used for logging)
        logging_path (str): The path to the logging directory.
        url (str): The URL of the server.
        port (int, optional): The port number to run the server on. Defaults to 8080.
    """

    def __init__(self, scene_name ="test", logging_path="logs", url="127.0.0.1", port=8080):
        """
        Initializes the NavConServer object.

        Args:
            scene_name (str): The name of the scene.
            logging_path (str): The path to the logging directory.
            url (str): The URL of the server.
            port (int, optional): The port number to run the server on. Defaults to 8080.
        """
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
        """
        Starts the server and waits for images.
        """
        print("Server started. Waiting for images.")
        self.app.run(debug=True, host=self.url, port=self.port, use_reloader=False)

    def upload(self):
        """
        Handles the image upload and text data submission.

        Returns:
            dict: The result of the navigation command.
        """
        img_bytes = request.files["image"].read()
        text_data = request.form["sentence"]
        self.recieved_prompt = text_data
        self.single_image = self.bytes_to_image(img_bytes, "image")
        result = self.get_nav_command()
        return jsonify(result)

    def get_nav_command(self):
        """
        Retrieves the navigation command based on the received prompt and image.

        Returns:
            str: The navigation command.
        """
        self.code = get_code(self.recieved_prompt)
        im = self.single_image
        self.pickle_data()
        result = execute_code(self.code, im, show_intermediate_steps=False)
        return result

    def bytes_to_image(self, img_bytes, frame):
        """
        Converts the image bytes to a numpy array.

        Args:
            img_bytes (bytes): The image bytes.
            frame (str): The name of the frame.

        Returns:
            torch.Tensor: The image tensor.
        """
        img_np = np.frombuffer(img_bytes, dtype=np.uint8)
        img = cv2.imdecode(img_np, flags=cv2.IMREAD_COLOR)
        transform = transforms.Compose([transforms.ToTensor()])
        return transform(img)

    def pickle_data(self):
        """
        Pickles the received prompt, code, and single image data.
        """
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

