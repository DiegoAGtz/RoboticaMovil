function sysCall_init()
    print('init')
end

function sysCall_actuation()
    -- put your actuation code here
    print('actuation')
end

function sysCall_sensing()
    print('sensing')
end

function getMap(size, scala, obj)
    map = {}
    i = 0
    for x = -size[1]/2, size[1]/2, scala do
        for y = -size[2]/2, size[2]/2, scala do
            sim.setObjectPosition(obj, -1, {x, y, 0.5})
            result, pairHandles = sim.checkCollision(obj, sim.handle_all)
            map[i] = result
	    i = i + 1
        end
    end
    
    return map
end

function sysCall_cleanup()
print('end')
    -- do some clean-up here
end