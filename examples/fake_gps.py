import airsim 
client = airsim.MultirotorClient("10.211.55.4")
state = client.getMultirotorState()
