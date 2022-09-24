

local types = require(script.types)

local function sign(num: number): number
    return if num < 0 then -1 else 1
end

local module = {}

function module.ComputeArc(point: CFrame, pointToMid: Vector3): types.arc
    local tangent = point.LookVector
    local point = point.Position

    local normal = pointToMid:Cross(tangent)
    local perpAxis = tangent:Cross(normal)

    local denominator = 2 * perpAxis:Dot(pointToMid)

    if math.abs(denominator) <= 0 then
        local center = point + (pointToMid * 0.5)

        return {
            center = center,
            radius = 0,
            angle = 0,
            axis1 = point - center,
            axis2 = tangent * 0,
            len = 0
        }

    else
        local centerDist = (pointToMid:Dot(pointToMid)) / denominator
        local center = point + (perpAxis * centerDist)

        local perpAxisMag = perpAxis.Magnitude
        local radius = math.abs(centerDist * perpAxisMag)

        local angle: number
        if radius <= 0 then
            angle = 0

        else
            local invRadius = 1 / radius

            local centerToMidDir = point - center
            local centerToEndDir = centerToMidDir * invRadius

            centerToMidDir += pointToMid
            centerToMidDir *= invRadius

            local twist = perpAxis:Dot(pointToMid)

            angle = math.acos(centerToEndDir:Dot(centerToMidDir)) * sign(twist)
        end

        return {
            center = center,
            radius = radius,
            angle = angle,
            axis1 = point - center,
            axis2 = tangent * radius,
            len = if radius == 0 then pointToMid.Magnitude else math.abs(radius * angle)
        }
    end
end

function module.ComputeBiarcs(p1: CFrame, p2: CFrame)
    local t1 = p1.LookVector
    local t2 = p2.LookVector

    local v = p2.Position - p1.Position

    local vDotV = v:Dot(v)

    if vDotV <= 0 then
        local arc1 = {
            center = p1.Position,
            radius = 0,
            axis1 = v,
            axis2 = v,
            angle = 0,
            len = 0
        }

        local arc2 = {
            center = p1.Position,
            radius = 0,
            axis1 = v,
            axis2 = v,
            angle = 0,
            len = 0
        }

        return arc1, arc2
    end

    local t = t1 + t2

    local vDotT = v:Dot(t)
    local t1DotT2 = t1:Dot(t2)
    local denominator = 2 * (1 - t1DotT2)

    local d: number
    if denominator <= 0 then
        local vDotT2 = v:Dot(t2)

        if math.abs(vDotT2) <= 0 then
            local vMag = math.sqrt(vDotV)
            local invVMagSqr = 1 / vDotV

            local planeNormal = v:Cross(t2)
            local perpAxis = planeNormal:Cross(v)

            local radius = vMag * 0.25
            local centerToP1 = v * -0.25

            local arc1 = {
                center = p1.Position - centerToP1,
                radius = radius,
                axis1 = centerToP1,
                axis2 = perpAxis * (radius * invVMagSqr),
                angle = math.pi,
                len = math.pi * radius
            }

            local arc2 = {
                center = p2.Position + centerToP1,
                radius = radius,
                axis1 = centerToP1 * -1,
                axis2 = perpAxis * (-radius * invVMagSqr),
                angle = math.pi,
                len = math.pi * radius
            }

            return arc1, arc2
        else
            d = vDotV / (4 * vDotT2)
        end

    else
        local discriminant = vDotT * vDotT + denominator * vDotV
        d = (-vDotT + math.sqrt(discriminant)) / denominator
    end

    local pm = ((p2.Position + (t1 - t2) * d) + p1.Position) * 0.5

    local p1ToPm = pm - p1.Position
    local p2ToPm = pm - p2.Position

    local arc1 = module.ComputeArc(p1, p1ToPm)
    local arc2 = module.ComputeArc(p2, p2ToPm)

    if d < 0 then
        arc1.angle = sign(arc1.angle) * (2 * math.pi) - arc1.angle
        arc2.angle = sign(arc2.angle) * (2 * math.pi) - arc2.angle
    end

    arc2.axis2 = t2 * -arc2.radius

    return arc1, arc2
end

function module.Interp(arc1: types.arc, arc2: types.arc, frac: number)
    local totalDist = arc1.len + arc2.len
    local fracDist = frac * totalDist

    if fracDist < arc1.len then
        if arc1.len <= 0 then
            return arc1.center + arc1.axis1

        else
            local arcFrac = fracDist / arc1.len
            if arc1.radius == 0 then
                return arc1.center + (arc1.axis1 * (-arcFrac * 2 + 1))

            else
                local angle = arc1.angle * arcFrac
                local sinRot = math.sin(angle)
                local cosRot = math.cos(angle)

                return arc1.center + (arc1.axis1 * cosRot) + (arc1.axis2 * sinRot)
            end
        end

    else
        if arc2.len <= 0 then
            return arc2.center + arc2.axis1

        else
            local arcFrac = (fracDist - arc1.len) / arc2.len

            if arc2.radius == 0 then
                return arc2.center + (arc2.axis1 * (arcFrac * 2 - 1))

            else
                local angle = arc2.angle * (1 - arcFrac)
                local sinRot = math.sin(angle)
                local cosRot = math.cos(angle)

                return arc2.center + (arc2.axis1 * cosRot) + (arc2.axis2 * sinRot)
            end
        end
    end
end

return module