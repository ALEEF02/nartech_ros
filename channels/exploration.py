from collections import deque

def shift_seen_grid(previous_grid, prev_width, prev_height, new_width, new_height, offset_x, offset_y, unseen_value=-1):
    if previous_grid is None:
        return [unseen_value] * (new_width * new_height)
    shifted = [unseen_value] * (new_width * new_height)
    max_idx = min(len(previous_grid), prev_width * prev_height)
    for idx in range(max_idx):
        cell_value = previous_grid[idx]
        if cell_value == unseen_value:
            continue
        x = idx % prev_width
        y = idx // prev_width
        nx = x + offset_x
        ny = y + offset_y
        if 0 <= nx < new_width and 0 <= ny < new_height:
            shifted[ny * new_width + nx] = 0
    return shifted


def _is_target_unknown(low_res_grid, camera_seen_grid, idx):
    if low_res_grid[idx] == -1:
        return True
    return camera_seen_grid is not None and camera_seen_grid[idx] == -1


def _is_traversable(low_res_grid, idx):
    return low_res_grid[idx] in (0, 127)


def BFS_for_nearest_unknown_cell(low_res_grid, new_width, new_height, start_x, start_y, camera_seen_grid=None):
    if low_res_grid is None or new_width <= 0 or new_height <= 0:
        return None
    if not (0 <= start_x < new_width and 0 <= start_y < new_height):
        return None
    if camera_seen_grid is not None and len(camera_seen_grid) != len(low_res_grid):
        camera_seen_grid = None
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    start_idx = start_y * new_width + start_x
    queue = deque([(start_x, start_y)])
    visited = {start_idx}
    while queue:
        x, y = queue.popleft()
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < new_width and 0 <= ny < new_height:
                nidx = ny * new_width + nx
                if _is_target_unknown(low_res_grid, camera_seen_grid, nidx):
                    return (nx, ny)
                if nidx not in visited and _is_traversable(low_res_grid, nidx):
                    queue.append((nx, ny))
                    visited.add(nidx)
    return None
