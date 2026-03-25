import os
import sys
import types


def _install_ros_stubs() -> None:
    if 'rclpy' not in sys.modules:
        sys.modules['rclpy'] = types.ModuleType('rclpy')

    if 'rclpy.node' not in sys.modules:
        node_module = types.ModuleType('rclpy.node')

        class DummyNode:
            pass

        node_module.Node = DummyNode
        sys.modules['rclpy.node'] = node_module

    if 'gen_bt_interfaces.srv' not in sys.modules:
        gen_srv_module = types.ModuleType('gen_bt_interfaces.srv')
        sys.modules['gen_bt_interfaces.srv'] = gen_srv_module
    gen_srv_module = sys.modules['gen_bt_interfaces.srv']

    class DummySrv:
        class Request:
            pass

        class Response:
            SUCCESS = 0
            RETRY = 1
            ERROR = 2

    gen_srv_module.CreatePayload = getattr(gen_srv_module, 'CreatePayload', DummySrv)
    gen_srv_module.PlanSubtree = getattr(gen_srv_module, 'PlanSubtree', DummySrv)
    gen_srv_module.SelectBehaviorTree = getattr(
        gen_srv_module, 'SelectBehaviorTree', DummySrv
    )


def _install_langchain_stubs() -> None:
    if 'langchain_core' not in sys.modules:
        sys.modules['langchain_core'] = types.ModuleType('langchain_core')
    if 'langchain_core.prompts' not in sys.modules:
        prompts_module = types.ModuleType('langchain_core.prompts')

        class DummyPromptTemplate:
            @staticmethod
            def from_template(template):
                return template

        prompts_module.PromptTemplate = DummyPromptTemplate
        sys.modules['langchain_core.prompts'] = prompts_module

    if 'langchain_core.messages' not in sys.modules:
        messages_module = types.ModuleType('langchain_core.messages')

        class DummyHumanMessage:
            def __init__(self, content):
                self.content = content

        messages_module.HumanMessage = DummyHumanMessage
        sys.modules['langchain_core.messages'] = messages_module

    if 'langchain_google_genai' not in sys.modules:
        llm_module = types.ModuleType('langchain_google_genai')

        class DummyLLM:
            init_calls = []

            def __init__(self, *args, **kwargs):
                type(self).init_calls.append(kwargs)

        llm_module.ChatGoogleGenerativeAI = DummyLLM
        sys.modules['langchain_google_genai'] = llm_module

    if 'langchain_openai' not in sys.modules:
        openai_module = types.ModuleType('langchain_openai')

        class DummyOpenAILLM:
            init_calls = []

            def __init__(self, *args, **kwargs):
                type(self).init_calls.append(kwargs)

        openai_module.ChatOpenAI = DummyOpenAILLM
        sys.modules['langchain_openai'] = openai_module

    if 'langchain_openrouter' not in sys.modules:
        openrouter_module = types.ModuleType('langchain_openrouter')

        class DummyOpenRouterLLM:
            init_calls = []

            def __init__(self, *args, **kwargs):
                type(self).init_calls.append(kwargs)

        openrouter_module.ChatOpenRouter = DummyOpenRouterLLM
        sys.modules['langchain_openrouter'] = openrouter_module


_install_ros_stubs()
_install_langchain_stubs()

from llm_interface.node import DEFAULT_PAYLOAD_PROMPT, LLMInterfaceNode


def test_extract_refinement_notes_prefers_structured_context():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    context = {
        'REQUEST_HINTS': {
            'MISSION_REFINEMENT': {
                'operator_feedback': 'the target is the second doorway',
                'prompt': 'Refine the mission payload to address the operator feedback above.',
                'prior_reasoning': 'The prior plan stopped at the first doorway.',
                'prior_payload_json': '{"waypoints": "1.0,2.0,0.0"}',
                'history': [
                    {
                        'iteration': 1,
                        'operator_feedback': 'the target is the second doorway',
                        'prior_payload_json': '{"waypoints": "1.0,2.0,0.0"}',
                    }
                ],
            }
        }
    }

    notes = node._extract_refinement_notes(
        context,
        'inspect hallway\n\nOPERATOR_REFINEMENT_FEEDBACK:\nthe target is the second doorway',
    )

    assert 'the target is the second doorway' in notes
    assert 'Refinement prompt:' in notes
    assert (
        node._extract_prior_plan_reasoning(context, 'inspect hallway')
        == 'The prior plan stopped at the first doorway.'
    )
    assert (
        node._extract_prior_plan_payload_json(context, 'inspect hallway')
        == '{"waypoints": "1.0,2.0,0.0"}'
    )
    assert '"iteration": 1' in node._extract_refinement_history_json(
        context, 'inspect hallway'
    )


def test_render_payload_prompt_includes_operator_refinement_notes():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    node._payload_prompts = {}
    node._payload_prompt_default = DEFAULT_PAYLOAD_PROMPT
    context = {
        'REQUEST_HINTS': {
            'MISSION_REFINEMENT': {
                'operator_feedback': 'use the waypoint near the loading ramp',
                'prior_reasoning': 'The prior payload missed the loading ramp waypoint.',
                'prior_payload_json': '{"waypoints": "0.0,0.0,0.0"}',
                'history': [
                    {
                        'iteration': 1,
                        'operator_feedback': 'missed loading ramp',
                        'prior_payload_json': '{"waypoints": "0.0,0.0,0.0"}',
                    }
                ],
            }
        }
    }

    prompt = node._render_payload_prompt(
        subtree_id='demo_tree.xml',
        user_command='inspect the loading zone',
        context=context,
        contract={'waypoints': {'type': 'string'}},
        attachment_uris=[],
    )

    assert 'OPERATOR_REFINEMENT_NOTES:' in prompt
    assert 'SESSION_REFINEMENT_HISTORY_JSON:' in prompt
    assert '"iteration": 1' in prompt
    assert 'use the waypoint near the loading ramp' in prompt
    assert 'PRIOR_REJECTED_PLAN_REASONING:' in prompt
    assert 'The prior payload missed the loading ramp waypoint.' in prompt
    assert 'PRIOR_REJECTED_PAYLOAD_JSON:' in prompt
    assert '{"waypoints": "0.0,0.0,0.0"}' in prompt
    assert '"REQUEST_HINTS"' in prompt


def test_prepare_llm_json_text_extracts_json_from_content_parts():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    raw_content = [
        {
            'type': 'text',
            'text': '{"tree_id": "demo_tree.xml", "confidence": 0.85}',
            'extras': {'signature': 'abc123'},
        }
    ]

    cleaned = node._prepare_llm_json_text(raw_content)

    assert cleaned == '{"tree_id": "demo_tree.xml", "confidence": 0.85}'


def test_prepare_llm_json_text_extracts_embedded_json_snippet():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)

    cleaned = node._prepare_llm_json_text(
        'Here is the payload:\n```json\n{"waypoints": "1.0,2.0,0.0"}\n```'
    )

    assert cleaned == '{"waypoints": "1.0,2.0,0.0"}'


def test_create_chat_llm_supports_openrouter_provider():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    node._openrouter_app_url = 'https://generalist.example'
    node._openrouter_app_title = 'Generalist BT'
    openrouter_cls = sys.modules['langchain_openrouter'].ChatOpenRouter
    openrouter_cls.init_calls.clear()
    original_key = os.environ.get('OPENROUTER_API_KEY')
    os.environ['OPENROUTER_API_KEY'] = 'test-openrouter-key'

    try:
        llm = node._create_chat_llm(
            provider_name='openrouter',
            model_name='openai/gpt-4o-mini',
            temperature=0.0,
            purpose='payload',
        )
    finally:
        if original_key is None:
            os.environ.pop('OPENROUTER_API_KEY', None)
        else:
            os.environ['OPENROUTER_API_KEY'] = original_key

    assert isinstance(llm, openrouter_cls)
    assert openrouter_cls.init_calls == [
        {
            'model': 'openai/gpt-4o-mini',
            'temperature': 0.0,
            'app_url': 'https://generalist.example',
            'app_title': 'Generalist BT',
        }
    ]


def test_create_chat_llm_supports_openai_provider():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    node._openrouter_app_url = ''
    node._openrouter_app_title = ''
    openai_cls = sys.modules['langchain_openai'].ChatOpenAI
    openai_cls.init_calls.clear()
    original_key = os.environ.get('OPENAI_API_KEY')
    os.environ['OPENAI_API_KEY'] = 'test-openai-key'

    try:
        llm = node._create_chat_llm(
            provider_name='openai',
            model_name='gpt-4o-mini',
            temperature=0.0,
            purpose='selection',
        )
    finally:
        if original_key is None:
            os.environ.pop('OPENAI_API_KEY', None)
        else:
            os.environ['OPENAI_API_KEY'] = original_key

    assert isinstance(llm, openai_cls)
    assert openai_cls.init_calls == [
        {
            'model': 'gpt-4o-mini',
            'temperature': 0.0,
        }
    ]


def test_normalize_provider_name_accepts_openrouter_aliases():
    assert LLMInterfaceNode._normalize_provider_name('google') == 'gemini'
    assert LLMInterfaceNode._normalize_provider_name('open_ai') == 'openai'
    assert LLMInterfaceNode._normalize_provider_name('open_router') == 'openrouter'
