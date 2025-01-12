from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
import os

model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
IMAGE_PATH = "sampled_images/set9/image1.jpg"
# TEXT_PROMPT = "detect pineapple slices on the white plate"
# TEXT_PROMPT = "detect cheese slices on wooden board"
# TEXT_PROMPT = "detect empty white plate"
# TEXT_PROMPT = "detect orange yellow scrambled eggs"
# TEXT_PROMPT = "detect white rice in a bowl"
# TEXT_PROMPT = "detect green cake"
TEXT_PROMPT = "detect white dumplings"
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

cv2.imwrite("inference_images/annotated_image.jpg", annotated_frame)