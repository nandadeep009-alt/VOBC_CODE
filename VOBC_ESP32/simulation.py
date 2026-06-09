limitOfAuthority = 50.0
civilSpeedLimit = 120.0
engineerMaxSpeed = 100.0
targetAcceleration = 10.0
targetDeceleration = 20.0
currentPosition = 0.0
currentVelocity = 0.0
status = 'RUNNING'

def stopping_distance(v, a):
    if a <= 0.0:
        return 999.0
    return (v * v) / (2.0 * a)

print('t(s),position,velocity,status')
dt = 0.1
for i in range(201):
    t = i * dt
    distanceToTarget = limitOfAuthority - currentPosition
    if distanceToTarget < 0.0:
        distanceToTarget += 100.0
    requiredStoppingDist = stopping_distance(currentVelocity, targetDeceleration)

    if distanceToTarget <= 0.1 or limitOfAuthority == 0.0:
        currentVelocity = 0.0
        status = 'IDLE'
    elif distanceToTarget <= requiredStoppingDist:
        status = 'RUNNING'
        currentVelocity -= targetDeceleration * dt
        if currentVelocity < 0.0:
            currentVelocity = 0.0
    else:
        status = 'RUNNING'
        targetCeiling = min(civilSpeedLimit, engineerMaxSpeed)
        if currentVelocity < targetCeiling:
            currentVelocity += targetAcceleration * dt
            if currentVelocity > targetCeiling:
                currentVelocity = targetCeiling
        elif currentVelocity > targetCeiling:
            currentVelocity -= targetDeceleration * dt
            if currentVelocity < targetCeiling:
                currentVelocity = targetCeiling

    currentPosition += (currentVelocity * 0.01) * dt
    if currentPosition >= 100.0:
        currentPosition -= 100.0

    print(f'{t:.1f},{currentPosition:.4f},{currentVelocity:.4f},{status}')
