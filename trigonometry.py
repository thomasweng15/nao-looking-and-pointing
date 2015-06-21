import math

# objectVector = [1, 1, 1] # vector from shoulder to object
#armVector = [0.058812521398067474, 0.1684359312057495, -0.0987471342086792] # vector from shoulder to end effector
# armVector values are currently torso to left hand, not shoulder 


# In meters
objectVector = [1, 1, 1] # vector from head to object
headVector = [1, 0, 0] # vector of head direction (zero pitch, zero yaw)

objectVectorMagn = math.sqrt(1^2 + 1^2 + 1^2)
objectUnitVector = [1 / objectVectorMagn, 1 / objectVectorMagn, 1 / objectVectorMagn]

# get required pitch and yaw so that robot will be looking at object
# TODO add headVector position to calculation
pitch = -math.asin(objectUnitVector[2])
yaw = math.asin(objectUnitVector[0])

print pitch, yaw


# Hand's position6d coordinates when arm is straightened,
# from the Torso coordinate frame
# [0.058812521398067474, 0.1684359312057495, -0.0987471342086792, 
# -0.7073619365692139, 1.1279035806655884, 0.8323026299476624]

# Shoulder's position, from the Torso coordinate frame
# [0.0, 0.09000000357627869, 0.10599999874830246, 0.0, -0.0, 0.0]

# Head's position, from the Torso coordinate frame
# [0.0, 0.0, 0.1264999955892563, 0.0, -0.0, 0.0]

# arm length: 0.227007700024
# rounded arm length: 0.225