import textwrap

from llm_interface.prompt_templates import (
    PromptRepository,
    TemplateNotFoundError,
)


def write_config(tmp_path, content):
    path = tmp_path / "config.yaml"
    path.write_text(textwrap.dedent(content), encoding="utf-8")
    return path


def test_render_prompt_with_defaults(tmp_path):
    path = write_config(
        tmp_path,
        """
        defaults:
          model: gpt-test
          temperature: 0.5
        prompts:
          llm_query:
            system: |
              use model {model}
            user: |
              task: {prompt}
              ctx: {context}
        """,
    )

    repo = PromptRepository.from_path(path)
    parts = repo.render(
        "llm_query",
        prompt="Drive to the barn",
        context="{}",
    )

    assert "use model gpt-test" in parts.system
    assert "Drive to the barn" in parts.user
    assert parts.metadata["variables"]["prompt"] == "Drive to the barn"
    assert repo.default_model == "gpt-test"


def test_missing_template_raises(tmp_path):
    path = write_config(
        tmp_path,
        """
        prompts:
          only_template:
            system: ""
            user: ""
        """,
    )

    repo = PromptRepository.from_path(path)
    try:
        repo.render("unknown")
    except TemplateNotFoundError as exc:
        assert "unknown" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("Expected TemplateNotFoundError")
