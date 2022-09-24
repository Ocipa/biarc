




export type arc = {
    center: Vector3,
    angle: number,
    radius: number,
    axis1: Vector3,
    axis2: Vector3,
    len: number,
}

export type biarc = {
    arc1: arc,
    arc2: arc,
    pm: Vector3,
}

export type line = {
    p1: Vector3,
    p2: Vector3,
}



return {}