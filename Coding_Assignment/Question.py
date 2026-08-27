def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    """
    Analyzes following distance compared to safe distance (speed * 0.5).
    
    Args:
        distances (list[float]): Distance to the lead car at each second.
        speeds (list[float]): Speed of our car at each second.
        
    Returns:
        tuple[int, float, int]: (tailgating_seconds, minimum_distance, tailgate_incidents)
            - tailgating_seconds: total seconds distance was < safe distance
            - minimum_distance: absolute closest distance to the lead car (return 0.0 if empty list)
            - tailgate_incidents: number of separate instances the car started tailgating
    """
    pass
distance = [30.0 , 20.0 , 20.0 , 30.0 , 15.0, 10.0]
speed = [50.0 , 50.0 , 50.0 , 50.0 , 50.0, 50.0]
global tailg_inci , tailg_sec , min_distance, first
first = True
tailg_sec= 0
min_distance = 0
tailg_inci = 0
def x(distance, speed):
    global tailg_inci , tailg_sec , min_distance, first
    for i , s in zip(distance, speed):
        safe_distance = 0.5 * s
        if i < safe_distance:
            tailg_sec += 1
            if first:
                min_distance = i
                first = False
                tailg_inci += 1
            if min_distance > i:
                min_distance = i
                tailg_inci += 1
    print(tailg_inci, tailg_sec, min_distance)
x(distance, speed)
