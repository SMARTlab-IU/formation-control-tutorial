import settings
import numpy as np
import cv2

U_ARROW_SCALE = 1.0
MAX_ARROW_LEN = 100


def global_error(positions, dstar):

    all_errors = []

    for i in range(settings.ROBOTS_N):
        for j in range(settings.ROBOTS_N):

            if i == j:
                continue

            rel = positions[j] - positions[i]
            desired = dstar[i, j]
            formation_error = rel - desired

            err_mag = np.linalg.norm(formation_error)
            all_errors.append(err_mag)

    if len(all_errors) > 0:
        return np.mean(all_errors)

    return 0


# Drawing functions created with AI assistance
def draw_text(frame, text, x, y, color=(255, 255, 255)):

    cv2.putText(
        frame,
        text,
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA
    )


def draw_u_arrow(frame, start, u, color=(0, 0, 255)):

    x = start[0]
    y = start[1]

    mag = np.linalg.norm(u)

    if mag < 1:
        return

    arrow = u * U_ARROW_SCALE

    arrow_len = np.linalg.norm(arrow)

    if arrow_len > MAX_ARROW_LEN:
        arrow = arrow / arrow_len * MAX_ARROW_LEN

    end = start + arrow

    cv2.arrowedLine(
        frame,
        (int(x), int(y)),
        (int(end[0]), int(end[1])),
        color,
        2,
        tipLength=0.25
    )


def draw_overlay(frame, ROBOTS, u, global_error):

    # Store which robot currently occupies each formation slot
    robot_by_slot = {}

    for robot in ROBOTS.values():

        if robot.slot is None:
            continue

        if robot.position is None:
            continue

        robot_by_slot[robot.slot] = robot

    # Draw current formation connections using assigned slot order
    for slot_i in range(settings.ROBOTS_N):

        next_slot_i = (slot_i + 1) % settings.ROBOTS_N

        if slot_i not in robot_by_slot:
            continue

        if next_slot_i not in robot_by_slot:
            continue

        robot_a = robot_by_slot[slot_i]
        robot_b = robot_by_slot[next_slot_i]

        cv2.line(
            frame,
            (int(robot_a.position[0]), int(robot_a.position[1])),
            (int(robot_b.position[0]), int(robot_b.position[1])),
            (120, 120, 120),
            2
        )

    draw_text(
        frame,
        f"global formation error: {global_error:.2f} px",
        20,
        30,
        (0, 255, 255)
    )

    # Draw robot positions, IDs, and current control vectors
    for robot in ROBOTS.values():

        if robot.position is None:
            continue

        i = robot.robot_i

        x = robot.position[0]
        y = robot.position[1]

        # Current robot position
        cv2.circle(
            frame,
            (int(x), int(y)),
            5,
            (255, 255, 255),
            -1
        )

        # Robot ID
        draw_text(
            frame,
            f"id {robot.marker_id}",
            x + 8,
            y - 8,
            (0, 255, 0)
        )

        # Current control vector
        draw_u_arrow(
            frame,
            robot.position,
            u[i],
            (0, 0, 255)
        )