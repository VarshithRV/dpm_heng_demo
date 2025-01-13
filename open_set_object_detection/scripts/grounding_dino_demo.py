from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
import os
from torchvision.ops import box_convert
import supervision as sv
import torch

model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
IMAGE_PATH = "sampled_images/set1/image1.jpg"
TEXT_PROMPT = "detect pineapple slices on the white plate"
# TEXT_PROMPT = "detect cheese slices on wooden board"
# TEXT_PROMPT = "detect empty white plate"
# TEXT_PROMPT = "detect orange yellow scrambled eggs"
# TEXT_PROMPT = "detect white rice in a bowl"
# TEXT_PROMPT = "detect green cake"
# TEXT_PROMPT = "detect white dumplings"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25

# image = cv2.imread(IMAGE_PATH)

image_source, image = load_image(IMAGE_PATH)

boxes, logits, phrases = predict(
    model=model,
    image=image,
    caption=TEXT_PROMPT,
    box_threshold=BOX_TRESHOLD,
    text_threshold=TEXT_TRESHOLD
)

annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)

print("########################")
print("\nBoxes : ")
print(boxes, type(boxes))
print("\n")

print("########################")
print("\nLogits : ")
print(logits, type(logits))
print("\n")

print("########################")
print("\nPhrases : ")
print(phrases, type(phrases))
print("\n")

max_distance = -1
max_index = -1

h, w, _ = image_source.shape
boxes = boxes * torch.Tensor([w, h, w, h])
xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy().astype(int)
detections = sv.Detections(xyxy=xyxy)
labels = [
    f"{phrase} {logit:.2f}"
    for phrase, logit
    in zip(phrases, logits)
]

for i in range(len(phrases)):
    x = int((xyxy[i][0] + xyxy[i][2]) / 2)
    y = int((xyxy[i][1] + xyxy[i][3]) / 2)
    distance = x**2 + y**2
    if distance > max_distance:
        max_distance = distance
    max_index = i

# Mark the x, y of the max on the annotated frame
if max_index != -1:
    x = int((xyxy[max_index][0] + xyxy[max_index][2]) / 2)
    y = int((xyxy[max_index][1] + xyxy[max_index][3]) / 2)
    cv2.circle(annotated_frame, (x, y), radius=10, color=(0, 0, 255), thickness=-1)

cv2.imwrite("inference_images/annotated_image.jpg", annotated_frame)