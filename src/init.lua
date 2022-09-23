

type tBiarcInterp_Arc = {
    m_center: Vector3,
    m_axis1: Vector3,
    m_axis2: Vector3,
    m_radius: number,
    m_angle: number,
    m_arcLen: number
}

local function sign(num: number): number
    return if num < 0 then -1 else 1
end

local module = {}

function module.BiarcInterp_ComputeArc(point: Vector3, tangent: Vector3, pointToMid: Vector3)
    local pCenter: Vector3
    local pRadius: number
    local pAngle: number

    local c_Epsilon = 0.0001

    local normal = pointToMid:Cross(tangent)
    local perpAxis = tangent:Cross(normal)

    local denominator = 2 * perpAxis:Dot(pointToMid)

    if math.abs(denominator) < c_Epsilon then
        pCenter = point + (pointToMid * 0.5)
        pRadius = 0
        pAngle = 0

    else
        local centerDist = (pointToMid:Dot(pointToMid)) / denominator
        pCenter = point + (perpAxis * centerDist)

        local perpAxisMag = perpAxis.Magnitude
        local radius = math.abs(centerDist * perpAxisMag)

        local angle: number
        if radius < c_Epsilon then
            angle = 0

        else
            local invRadius = 1 / radius

            local centerToMidDir = point - pCenter
            local centerToEndDir = centerToMidDir * invRadius

            centerToMidDir += pointToMid
            centerToMidDir *= invRadius

            local twist = perpAxis:Dot(pointToMid)

            angle = math.acos(centerToEndDir:Dot(centerToMidDir)) * sign(twist)
        end

        pRadius = radius
        pAngle = angle
    end

    return pCenter, pRadius, pAngle
end

function module.BiarcInterp_ComputeArcs(p1: Vector3, t1: Vector3, p2: Vector3, t2: Vector3)
    local pArc1: tBiarcInterp_Arc = {}
    local pArc2: tBiarcInterp_Arc = {}

    local c_Epsilon = 0.0001

    local v = p2 - p1

    local vDotV = v:Dot(v)

    if vDotV < c_Epsilon then
        pArc1.m_center = p1
        pArc1.m_radius = 0
        pArc1.m_axis1 = v
        pArc1.m_axis2 = v
        pArc1.m_angle = 0
        pArc1.m_arcLen = 0

        pArc2.m_center = p1
        pArc2.m_radius = 0
        pArc2.m_axis1 = v
        pArc2.m_axis2 = v
        pArc2.m_angle = 0
        pArc2.m_arcLen = 0

        return pArc1, pArc2
    end

    local t = t1 + t2

    local vDotT = v:Dot(t)
    local t1DotT2 = t1:Dot(t2)
    local denominator = 2 * (1 - t1DotT2)

    local d: number
    if denominator < c_Epsilon then
        local vDotT2 = v:Dot(t2)

        if math.abs(vDotT2) < c_Epsilon then
            local vMag = math.sqrt(vDotV)
            local invVMagSqr = 1 / vDotV

            local planeNormal = v:Cross(t2)
            local perpAxis = planeNormal:Cross(v)

            local radius = vMag * 0.25
            local centerToP1 = v * -0.25

            pArc1.m_center = p1 - centerToP1
            pArc1.m_radius = radius
            pArc1.m_axis1 = centerToP1
            pArc1.m_axis2 = perpAxis * (radius * invVMagSqr)
            pArc1.m_angle = math.pi
            pArc1.m_arcLen = math.pi * radius

            pArc2.m_center = p2 + centerToP1
            pArc2.m_radius = radius
            pArc2.m_axis1 = centerToP1 * -1
            pArc2.m_axis2 = perpAxis * (-radius * invVMagSqr)
            pArc2.m_angle = math.pi
            pArc2.m_arcLen = math.pi * radius

            return pArc1, pArc2
        else
            d = vDotV / (4 * vDotT2)
        end

    else
        local discriminant = vDotT * vDotT + denominator * vDotV
        d = (-vDotT + math.sqrt(discriminant)) / denominator
    end

    local pm = ((p2 + (t1 - t2) * d) + p1) * 0.5

    local p1ToPm = pm - p1
    local p2ToPm = pm - p2

    local center1, radius1, angle1 = module.BiarcInterp_ComputeArc(p1, t1, p1ToPm)
    local center2, radius2, angle2 = module.BiarcInterp_ComputeArc(p2, t2, p2ToPm)

    if d < 0 then
        angle1 = sign(angle1) * (2 * math.pi) - angle1
        angle2 = sign(angle2) * (2 * math.pi) - angle2
    end

    pArc1.m_center = center1
    pArc1.m_radius = radius1
    pArc1.m_axis1 = p1 - center1
    pArc1.m_axis2 = t1 * radius1
    pArc1.m_angle = angle1
    pArc1.m_arcLen = if radius1 == 0 then p1ToPm.Magnitude else math.abs(radius1 * angle1)

    pArc2.m_center = center2
    pArc2.m_radius = radius2
    pArc2.m_axis1 = p2 - center2
    pArc2.m_axis2 = t2 * -radius2
    pArc2.m_angle = angle2
    pArc2.m_arcLen = if radius2 == 0 then p2ToPm.Magnitude else math.abs(radius2 * angle2)

    return pArc1, pArc2
end

function module.BiarcInterp(arc1: tBiarcInterp_Arc, arc2: tBiarcInterp_Arc, frac: number)
    local result: Vector3

    local epsilon = 0.0001

    local totalDist = arc1.m_arcLen + arc2.m_arcLen
    local fracDist = frac * totalDist

    if fracDist < arc1.m_arcLen then
        if arc1.m_arcLen < epsilon then
            result = arc1.m_center + arc1.m_axis1

        else
            local arcFrac = fracDist / arc1.m_arcLen
            if arc1.m_radius == 0 then
                result = arc1.m_center + (arc1.m_axis1 * (-arcFrac * 2 + 1))

            else
                local angle = arc1.m_angle * arcFrac
                local sinRot = math.sin(angle)
                local cosRot = math.cos(angle)

                result = arc1.m_center + (arc1.m_axis1 * cosRot)
                result = result + (arc1.m_axis2 * sinRot)
            end
        end

    else
        if arc2.m_arcLen < epsilon then
            result = arc2.m_center + arc2.m_axis1

        else
            local arcFrac = (fracDist - arc1.m_arcLen) / arc2.m_arcLen

            if arc2.m_radius == 0 then
                result = arc2.m_center + (arc2.m_axis1 * (arcFrac * 2 - 1))

            else
                local angle = arc2.m_angle * (1 - arcFrac)
                local sinRot = math.sin(angle)
                local cosRot = math.cos(angle)

                result = arc2.m_center + (arc2.m_axis1 * cosRot)
                result = result + (arc2.m_axis2 * sinRot)
            end
        end
    end

    return result
end

return module