"""
Minimal LLaVA FastAPI server.
Loads LLaVA-1.6-Mistral-7B in 4-bit and answers questions about posted images.
POST /describe  with an image file and optional 'prompt' form field.
"""
import io
import torch
from fastapi import FastAPI, UploadFile, File, Form
from PIL import Image
from transformers import LlavaNextProcessor, LlavaNextForConditionalGeneration
from transformers import BitsAndBytesConfig

MODEL_ID = "llava-hf/llava-v1.6-mistral-7b-hf"

app = FastAPI()

print("Loading LLaVA (4-bit). First run downloads ~14 GB, please wait...")

quant_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_compute_dtype=torch.float16,
    bnb_4bit_quant_type="nf4",
)

processor = LlavaNextProcessor.from_pretrained(MODEL_ID)
model = LlavaNextForConditionalGeneration.from_pretrained(
    MODEL_ID,
    quantization_config=quant_config,
    torch_dtype=torch.float16,
    device_map="auto",
)
print("LLaVA loaded and ready.")


@app.get("/health")
def health():
    return {"status": "ok"}


@app.post("/describe")
async def describe(
    image: UploadFile = File(...),
    prompt: str = Form("Describe what you see. Is there a person? What objects are present?"),
):
    raw = await image.read()
    pil = Image.open(io.BytesIO(raw)).convert("RGB")

    conversation = [
        {"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image"},
        ]},
    ]
    chat = processor.apply_chat_template(conversation, add_generation_prompt=True)
    inputs = processor(images=pil, text=chat, return_tensors="pt").to(model.device)

    with torch.no_grad():
        out = model.generate(**inputs, max_new_tokens=120, do_sample=False)

    text = processor.decode(out[0], skip_special_tokens=True)
    # Keep only the model's answer (after the last assistant turn marker)
    if "[/INST]" in text:
        text = text.split("[/INST]")[-1].strip()
    return {"description": text}
