ul, ur = 2, 2
    lgains = np.linspace(0, -1, len(upt))
    rgains = np.linspace(-1, 0, len(upt))
    for k in range(len(upt)):
        if ustate[k]:
            ul = ul + lgains[k] * (1.0 - uread[k])
            ur = ur + rgains[k] * (1.0 - uread[k])

    if (
        (uread[2] < 0.3 and uread[2] > 0)
        or (uread[3] < 0.5 and uread[3] > 0)
        or (uread[4] < 0.5 and uread[4] > 0)
        or (uread[5] < 0.3 and uread[5] > 0)
    ):
        t_tmp = time.time()
        while time.time() - t_tmp < 0.5:
            ul, ur = -2, -2
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
        degr = random.randint(45, 135) * random.choice([-1, 1])
        sign = random.choice([-1, 1])
        print(f"Grados a dar: {degr} - Segundos: {0.5 * (degr // 30)}")
        t_tmp = time.time()
        while time.time() - t_tmp < 0.5 * (degr // 30):
            ur, ul = v2u(0, sign * 2.0 * m.pi / 12.0, r, L)
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
    else:
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)
        niter = niter + 1
