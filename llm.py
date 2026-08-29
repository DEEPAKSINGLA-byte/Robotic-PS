import os
import json
import threading
from math import sqrt
from typing import Optional
from groq import Groq
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

JSON_FILE = 'json/final_map.json'
MAX_CANDIDATES = 20
NEAR_THRESHOLD = 2.0
MODEL = 'openai/gpt-oss-20b'

env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.env')
if os.path.exists(env_path):
    with open(env_path, 'r') as f:
        for line in f:
            if line.strip() and not line.startswith('#'):
                key, val = line.strip().split('=', 1)
                os.environ[key.strip()] = val.strip()

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
            
    parser_schema = {
        'type': 'object',
        'properties': {
            'action': {'type': ['string', 'null'], 'enum': ['navigate_to', 'traverse', None]},
            'target_class': {'type': ['string', 'null']},
            'reference_class': {'type': ['string', 'null']},
            'relation': {'type': ['string', 'null'], 'enum': ['near', None]}
        },
        'required': ['action', 'target_class', 'reference_class', 'relation'],
        'additionalProperties': False
    }
    
    system_prompt = (
        "\nYou are the semantic command parser for a mobile robot.\n"
        "Your job is to identify the action, target object, and optional spatial reference.\n"
        f"Available object classes:\n{json.dumps(available_classes)}\n"
        f"Previous target:\n{json.dumps(previous_target_data) if previous_target_data is not None else 'null'}\n"
        "Rules:\n"
        "1. For commands such as 'go to', 'move to', 'navigate to', 'approach', set action = 'navigate_to'.\n"
        "2. For commands such as 'go through', 'pass through', 'traverse', set action = 'traverse'.\n"
        "3. target_class must be one of the available object classes, unless no target can be identified, in which case use null.\n"
        "4. reference_class must be one of the available object classes, or null.\n"
        "5. If the command expresses proximity ('near', 'beside', 'next to', 'close to', 'by'), set relation to 'near'.\n"
        "6. If the user uses a pronoun ('it', 'that', 'this object') and a previous target exists, resolve the target to the previous target's class.\n"
        "7. If you cannot determine the target object, set action, target_class, reference_class, and relation all to null.\n"
        "8. Do not invent classes.\n"
        "Return only the requested JSON structure.\n"
    )
    
    response = client.chat.completions.create(
        model=MODEL,
        messages=[{'role': 'system', 'content': system_prompt}, {'role': 'user', 'content': command}],
        temperature=0,
        response_format={'type': 'json_schema', 'json_schema': {'name': 'command_query', 'strict': True, 'schema': parser_schema}}
    )
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
        
        if not candidates:
            return []
            
        unique_candidates = {}
        for candidate in candidates:
            object_id = candidate['target']['object_id']
            if object_id not in unique_candidates:
                unique_candidates[object_id] = candidate
                
        candidates = list(unique_candidates.values())
        
        if robot_position is not None:
            for candidate in candidates:
                target_obj = get_object_by_id(objects, candidate['target']['object_id'])
                candidate['robot_distance'] = distance_from_robot(target_obj)
                
            candidates.sort(
                key=lambda x: x['robot_distance'] if x['robot_distance'] is not None else float('inf')
            )
            
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
    
    if not query.get('action') or not query.get('target_class'):
        return {'success': False, 'error': 'I could not determine the target object or action.', 'query': query}
        
    candidates = generate_candidates(objects, query, NEAR_THRESHOLD)
    if not candidates:
        return {'success': False, 'error': 'No matching candidates found.', 'query': query}
        
    best_candidate = candidates[0]
    object_id = best_candidate['target']['object_id']
    
    task = {
        'action': query['action'],
        'object_id': object_id
    }
    
    resolved = resolve_task(task, objects)
    update_context(command, task)
    return {'success': True, 'command': command, 'query': query, 'candidates': candidates, 'task': task, 'resolved_object': resolved, 'context': context}

def ros_spin_thread(node):
    rclpy.spin(node)

if __name__ == '__main__':
    print(f'Loaded {len(objects)} objects.')
    print('Available classes:')
    print(available_classes)
    print('\nRobot command interface.')
    print("Type 'exit' to quit.\n")
    
    rclpy.init()
    node = rclpy.create_node('semantic_llm_commander')
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    
    def odom_callback(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        set_robot_position([x, y, z])
        
    odom_sub = node.create_subscription(Odometry, '/odom', odom_callback, 10)
    
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    while True:
        command = input('Command > ').strip()
        if command.lower() in {'exit', 'quit'}:
            break
        if not command:
            continue
        try:
            result = process_command(command)
            print(json.dumps(result, indent=2))
            if result.get('success'):
                task = result.get('task')
                resolved = result.get('resolved_object')
                if task and task.get('action') == 'navigate_to':
                    goal = resolved.get('safe_nav_goal')
                    if not goal:
                        goal = resolved.get('centroid')
                    if goal:
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = node.get_clock().now().to_msg()
                        pose_msg.header.frame_id = 'map'
                        pose_msg.pose.position.x = float(goal[0])
                        pose_msg.pose.position.y = float(goal[1])
                        pose_msg.pose.position.z = 0.0
                        pose_msg.pose.orientation.w = 1.0
                        goal_pub.publish(pose_msg)
                        print(f"[Navigator] Published goal to /goal_pose: x={goal[0]:.2f}, y={goal[1]:.2f}")
                    else:
                        print("[Navigator] Error: No safe_nav_goal or centroid found for the target.")
            else:
                print(f"[Navigator] {result.get('error')}")
        except Exception as e:
            print(json.dumps({'success': False, 'error': str(e)}, indent=2))
            
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()