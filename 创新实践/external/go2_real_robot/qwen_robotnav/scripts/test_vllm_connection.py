"""Test connectivity to the remote vLLM/Qwen3-VL service."""

import argparse
import os
import sys

from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-url",
        default=os.environ.get("VLLM_BASE_URL", "http://100.113.176.95:8000/v1"),
    )
    parser.add_argument(
        "--model",
        default=os.environ.get("VLLM_MODEL", "qwen3-vl-fp8"),
    )
    parser.add_argument("--api-key", default=os.environ.get("VLLM_API_KEY") or "not-needed")
    args = parser.parse_args()

    try:
        import openai
    except ImportError:
        print("ERROR: openai package is not installed.", file=sys.stderr)
        sys.exit(1)

    client = openai.OpenAI(base_url=args.base_url, api_key=args.api_key, timeout=60)

    print(f"Pinging {args.base_url} ...")
    models = client.models.list()
    model_ids = [m.id for m in models.data]
    print(f"Available models: {model_ids}")

    if args.model not in model_ids:
        print(f"WARNING: requested model {args.model} not in list; using first available.")
        args.model = model_ids[0] if model_ids else args.model

    # Create a tiny dummy image.
    image = Image.new("RGB", (224, 224), (100, 150, 200))

    import base64
    from io import BytesIO

    buffer = BytesIO()
    image.save(buffer, format="PNG")
    b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
    data_url = f"data:image/png;base64,{b64}"

    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": data_url}},
                {
                    "type": "text",
                    "text": (
                        "Describe what you see in one sentence, then return a JSON array "
                        "of 8 waypoints [{x,y,theta}] for a robot moving forward."
                    ),
                },
            ],
        }
    ]

    print(f"Sending chat request to model={args.model} ...")
    response = client.chat.completions.create(
        model=args.model,
        messages=messages,
        max_tokens=512,
        temperature=0.2,
    )
    choice = response.choices[0]
    message = choice.message
    print("\n--- RESPONSE ---")
    print(message.content)
    print(f"finish_reason={choice.finish_reason}")
    reasoning_content = getattr(message, "reasoning_content", None)
    if reasoning_content:
        print("--- REASONING CONTENT ---")
        print(reasoning_content)
    print("----------------")
    print("Connection test succeeded.")


if __name__ == "__main__":
    main()
