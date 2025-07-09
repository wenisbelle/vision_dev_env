from ultralytics import YOLO

# Load a model
model = YOLO("best.pt")  # load a custom model

# Predict with the model
for i in range(1, 7):
    results = model(f"./data_for_test/{i}.png", save=True)  # predict on an image
    print(f"Results for image {i}: {results}")


