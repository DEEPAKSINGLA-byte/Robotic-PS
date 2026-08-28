import os
import json
from math import sqrt
from typing import Optional
from groq import Groq
JSON_FILE = 'world.json'
MAX_CANDIDATES = 20
NEAR_THRESHOLD = 2.0
MODEL = 'openai/gpt-oss-20b'
api_key = os.environ.get('GROQ_API_KEY')
if not api_key:
    raise RuntimeError('GROQ_API_KEY environment variable is not set.')
client = Groq(api_key=api_key)
with open(JSON_FILE, 'r') as f:
    objects = json.load(f)
if not isinstance(objects, list):
    raise ValueError('world.json must contain a list of objects.')
available_classes = sorted({obj['class_name'].lower() for obj in objects if 'class_name' in obj})
context = {'previous_target': None, 'previous_command': None, 'current_task': None}
robot_position = None

def set_robot_position(position):
    global robot_position
    if not isinstance(position, (list, tuple)):
        raise ValueError('Robot position must be a list or tuple.')
    if len(position) < 2:
        raise ValueError('Robot position must contain at least x and y.')
    robot_position = list(position)

def retrieve_objects(objects, class_name):
    results = []
    for obj in objects:
        if 'class_name' not in obj:
            continue
        if obj['class_name'].lower() == class_name.lower():
            results.append(obj)
    return results

def simplify_object(obj):
    result = {'object_id': obj['object_id'], 'class_name': obj['class_name'], 'centroid': obj['centroid']}
    if 'safe_nav_goal' in obj:
        result['safe_nav_goal'] = obj['safe_nav_goal']
    else:
        result['safe_nav_goal'] = None
    return result

def build_context(objects, relevant_classes):
    context_data = {}
    for class_name in relevant_classes:
        matches = retrieve_objects(objects, class_name)
        simplified_matches = []
        for obj in matches:
            simplified_matches.append(simplify_object(obj))
        context_data[class_name] = simplified_matches
    return context_data

def distance_between(obj_a, obj_b):
    a = obj_a['centroid']
    b = obj_b['centroid']
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    if len(a) >= 3 and len(b) >= 3:
        dz = a[2] - b[2]
    else:
        dz = 0.0
    return sqrt(dx ** 2 + dy ** 2 + dz ** 2)

def distance_from_robot(obj):
    if robot_position is None:
        return None
    centroid = obj['centroid']
    dx = centroid[0] - robot_position[0]
    dy = centroid[1] - robot_position[1]
    if len(centroid) >= 3 and len(robot_position) >= 3:
        dz = centroid[2] - robot_position[2]
    else:
        dz = 0.0
    return sqrt(dx ** 2 + dy ** 2 + dz ** 2)

def find_near_objects(objects_a, objects_b, threshold):
    results = []
    for obj_a in objects_a:
        for obj_b in objects_b:
            distance = distance_between(obj_a, obj_b)
            if distance <= threshold:
                results.append({'object_a': obj_a, 'object_b': obj_b, 'distance': distance})
    return results

def filter_by_spatial_relation(objects, target_class, reference_class, relation, threshold):
    target_objects = retrieve_objects(objects, target_class)
    reference_objects = retrieve_objects(objects, reference_class)
    if relation == 'near':
        pairs = find_near_objects(target_objects, reference_objects, threshold)
        candidates = []
        for pair in pairs:
            target = simplify_object(pair['object_a'])
            reference = simplify_object(pair['object_b'])
            candidates.append({'target': target, 'reference': reference, 'distance': pair['distance']})
        candidates.sort(key=lambda x: x['distance'])
        return candidates
    return []

def find_classes_in_command(command, available_classes):
    command = command.lower()
    found_classes = []
    for class_name in available_classes:
        if class_name in command:
            found_classes.append(class_name)
    return found_classes

def parse_command(command):
    previous_target = context['previous_target']
    if previous_target is None:
        previous_target_data = None
    else:
        previous_target_data = get_object_by_id(objects, previous_target)
        if previous_target_data is not None:
            previous_target_data = simplify_object(previous_target_data)
    parser_schema = {'type': 'object', 'properties': {'target_class': {'type': ['string', 'null']}, 'reference_class': {'type': ['string', 'null']}, 'relation': {'type': ['string', 'null'], 'enum': ['near', None]}}, 'required': ['target_class', 'reference_class', 'relation'], 'additionalProperties': False}
    system_prompt = '\nYou are the semantic command parser for a mobile robot.\nYour job is to identify the target object and optional spatial\nreference from the user\'s command.\nAvailable object classes:\n%s\nPrevious target:\n%s\nRules:\n1. target_class must be one of the available object classes,\n   unless no target can be identified, in which case use null.\n2. reference_class must be one of the available object classes,\n   or null.\n3. If the command expresses proximity such as:\n   near, beside, next to, close to, by\n   set relation to "near".\n4. If the user uses a pronoun such as:\n   it, that, this object\n   and a previous target exists, resolve the target to the\n   previous target\'s class.\n5. Do not invent classes.\nReturn only the requested JSON structure.\n' % (json.dumps(available_classes), json.dumps(previous_target_data) if previous_target_data is not None else 'null')
    response = client.chat.completions.create(model=MODEL, messages=[{'role': 'system', 'content': system_prompt}, {'role': 'user', 'content': command}], temperature=0, response_format={'type': 'json_schema', 'json_schema': {'name': 'command_query', 'strict': True, 'schema': parser_schema}})
    content = response.choices[0].message.content
    query = json.loads(content)
    return query

def get_object_by_id(objects, object_id):
    for obj in objects:
        if obj.get('object_id') == object_id:
            return obj
    return None

def generate_candidates(objects, query, near_threshold=NEAR_THRESHOLD):
    target_class = query.get('target_class')
    reference_class = query.get('reference_class')
    relation = query.get('relation')
    if target_class is None:
        return []
    target_class = target_class.lower()
    if target_class not in available_classes:
        return []
    if reference_class is not None and relation is not None:
        reference_class = reference_class.lower()
        if reference_class not in available_classes:
            return []
        candidates = filter_by_spatial_relation(objects, target_class, reference_class, relation, near_threshold)
        return candidates[:MAX_CANDIDATES]
    target_objects = retrieve_objects(objects, target_class)
    candidates = []
    for obj in target_objects:
        simplified = simplify_object(obj)
        robot_distance = distance_from_robot(obj)
        candidates.append({'target': simplified, 'robot_distance': robot_distance})
    if robot_position is not None:
        candidates.sort(key=lambda x: x['robot_distance'] if x['robot_distance'] is not None else float('inf'))
    return candidates[:MAX_CANDIDATES]

def select_task(command, query, candidates):
    if not candidates:
        raise RuntimeError('No valid object candidates found.')
    candidate_json = json.dumps(candidates, indent=2)
    previous_target = context['previous_target']
    task_schema = {'type': 'object', 'properties': {'action': {'type': 'string', 'enum': ['navigate_to', 'traverse']}, 'object_id': {'type': 'integer'}}, 'required': ['action', 'object_id'], 'additionalProperties': False}
    system_prompt = '\nYou are the final task-selection module for a mobile robot.\nThe user gave this command:\n%s\nParsed semantic query:\n%s\nPrevious target object ID:\n%s\nAvailable candidates:\n%s\nSelect exactly one object.\nRules:\n1. object_id MUST come from the candidate list.\n2. Never invent an object_id.\n3. For commands such as:\n   "go to"\n   "move to"\n   "navigate to"\n   "approach"\n   use:\n   action = "navigate_to"\n4. For commands such as:\n   "go through"\n   "pass through"\n   "traverse"\n   use:\n   action = "traverse"\n5. For "it", "that", or similar references, use the\n   previous target when it is the appropriate candidate.\n6. Return only the requested JSON.\n' % (command, json.dumps(query), json.dumps(previous_target), candidate_json)
    response = client.chat.completions.create(model=MODEL, messages=[{'role': 'system', 'content': system_prompt}, {'role': 'user', 'content': command}], temperature=0, response_format={'type': 'json_schema', 'json_schema': {'name': 'robot_task', 'strict': True, 'schema': task_schema}})
    content = response.choices[0].message.content
    task = json.loads(content)
    return task

def validate_task(task, candidates):
    if not isinstance(task, dict):
        raise ValueError('LLM task is not a dictionary.')
    if 'action' not in task:
        raise ValueError('LLM task missing action.')
    if 'object_id' not in task:
        raise ValueError('LLM task missing object_id.')
    action = task['action']
    object_id = task['object_id']
    allowed_actions = {'navigate_to', 'traverse'}
    if action not in allowed_actions:
        raise ValueError(f'Invalid action: {action}')
    candidate_ids = set()
    for candidate in candidates:
        if 'target' in candidate:
            candidate_ids.add(candidate['target']['object_id'])
    if object_id not in candidate_ids:
        raise ValueError('LLM returned an object_id that was not present in the candidate list.')
    return True

def resolve_task(task, objects):
    object_id = task['object_id']
    obj = get_object_by_id(objects, object_id)
    if obj is None:
        raise RuntimeError(f'Object ID {object_id} no longer exists in the local world model.')
    result = {'action': task['action'], 'object_id': object_id, 'class_name': obj['class_name'], 'centroid': obj['centroid']}
    if 'safe_nav_goal' in obj:
        result['safe_nav_goal'] = obj['safe_nav_goal']
    else:
        result['safe_nav_goal'] = None
    if task['action'] == 'traverse':
        if 'min_bound' in obj:
            result['min_bound'] = obj['min_bound']
        else:
            result['min_bound'] = None
        if 'max_bound' in obj:
            result['max_bound'] = obj['max_bound']
        else:
            result['max_bound'] = None
    return result

def update_context(command, task):
    context['previous_command'] = command
    context['previous_target'] = task['object_id']
    context['current_task'] = task

def process_command(command):
    query = parse_command(command)
    candidates = generate_candidates(objects, query, NEAR_THRESHOLD)
    if not candidates:
        return {'success': False, 'error': 'No matching candidates found.', 'query': query}
    task = select_task(command, query, candidates)
    validate_task(task, candidates)
    resolved = resolve_task(task, objects)
    update_context(command, task)
    return {'success': True, 'command': command, 'query': query, 'candidates': candidates, 'task': task, 'resolved_object': resolved, 'context': context}
if __name__ == '__main__':
    print(f'Loaded {len(objects)} objects.')
    print('Available classes:')
    print(available_classes)
    print('\nRobot command interface.')
    print("Type 'exit' to quit.\n")
    while True:
        command = input('Command > ').strip()
        if command.lower() in {'exit', 'quit'}:
            break
        if not command:
            continue
        try:
            result = process_command(command)
            print(json.dumps(result, indent=2))
        except Exception as e:
            print(json.dumps({'success': False, 'error': str(e)}, indent=2))