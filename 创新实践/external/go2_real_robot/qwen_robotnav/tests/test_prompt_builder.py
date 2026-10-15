"""Unit tests for prompt construction."""

from PIL import Image

from qwen_robotnav import NavigationConfig, PromptBuilder, TaskMode
from qwen_robotnav.observation_encoder import AllocatedFrame, ObservationEncoder


def test_system_prompt_contains_embodiment():
    cfg = NavigationConfig(embodiment="quadruped robot")
    builder = PromptBuilder(cfg)
    prompt = builder.build_system_prompt()
    assert "quadruped robot" in prompt
    assert "waypoint" in prompt


def test_user_text_with_images():
    cfg = NavigationConfig(
        task_mode=TaskMode.OBJNAV,
        instruction="find the kitchen",
    )
    builder = PromptBuilder(cfg)
    allocated = [
        AllocatedFrame(Image.new("RGB", (64, 64), "red"), "Front", 0, 16, 2.0),
        AllocatedFrame(Image.new("RGB", (64, 64), "blue"), "Back", 0, 4, 0.5),
    ]
    text = builder.build_user_text(allocated)
    assert "ObjNav" in text
    assert "kitchen" in text
    assert "Time step 0" in text
    assert "Front camera: <image>" in text
    assert "Back camera: <image>" in text


def test_messages_structure():
    cfg = NavigationConfig()
    builder = PromptBuilder(cfg)
    allocated = [
        AllocatedFrame(Image.new("RGB", (64, 64), "red"), "Front", 0, 16, 2.0),
    ]
    messages, images = builder.build_messages(allocated)
    assert len(messages) == 2
    assert messages[0]["role"] == "system"
    assert messages[1]["role"] == "user"
    assert len(images) == 1


def test_empty_frames_fallback():
    cfg = NavigationConfig(task_mode=TaskMode.POINTNAV, instruction="go to (3, 2)")
    builder = PromptBuilder(cfg)
    text = builder.build_user_text([])
    assert "PointNav" in text
    assert "go to (3, 2)" in text
    assert "No visual observations" in text
