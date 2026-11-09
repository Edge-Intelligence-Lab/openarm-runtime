"""Pure, camera-view pose illustrations; recommendations, never measurements."""
from dataclasses import dataclass
from html import escape
import math


@dataclass(frozen=True)
class Pose:
    title: str
    instruction: str
    yaw: float = 0
    pitch: float = 0
    x: float = .5
    y: float = .5
    scale: float = 1


FIT_POSES = (
    Pose('正面 · 中心','板正对相机，放在画面中心。'),
    Pose('正面 · 偏左','不转板面，把板移到画面左侧。',x=.25),
    Pose('正面 · 偏右','不转板面，把板移到画面右侧。',x=.75),
    Pose('正面 · 稍远','仍放在中心，比第一张稍远；保持标记清晰。',scale=.75),
    Pose('左边靠近相机','像开门一样，让画面中板的左边靠近镜头，约 30°。',yaw=-28),
    Pose('左边靠近 · 偏左上','保持左边近、右边远，把板移到画面左上区域。',yaw=-32,x=.35,y=.3),
    Pose('右边靠近相机','反过来，让画面中板的右边靠近镜头，约 30°。',yaw=28),
    Pose('右边靠近 · 偏右下','保持右边近、左边远，把板移到画面右下区域。',yaw=32,x=.65,y=.7),
    Pose('上边靠近相机','让板的上边靠近镜头、下边远离，约 30°。',pitch=-28),
    Pose('上边靠近 · 偏右上','保持上边近、下边远，把板移到画面右上区域。',pitch=-32,x=.65,y=.3),
    Pose('下边靠近相机','反过来，让板的下边靠近镜头，约 30°。',pitch=28),
    Pose('下边靠近 · 偏左下','保持下边近、上边远，把板移到画面左下区域。',pitch=32,x=.35,y=.7),
    Pose('左上角靠近相机','同时左右、上下倾斜，让左上角最近；两个方向各约 25°。',yaw=-24,pitch=-24,x=.35,y=.35),
    Pose('右上角靠近相机','换成右上角最近，保持两方向都有倾斜。',yaw=24,pitch=-24,x=.65,y=.35),
    Pose('左下角靠近相机','换成左下角最近；不要只在画面里转圈。',yaw=-24,pitch=24,x=.35,y=.65),
    Pose('右下角靠近相机','最后换成右下角最近，停稳后拍摄。',yaw=24,pitch=24,x=.65,y=.65),
)
VALIDATION_POSES = (
    Pose('检查 · 正面稍远','换一个距离和位置，拍正面；这张不参与拟合。',x=.4,scale=.85),
    Pose('检查 · 右边靠近','右边靠近镜头，约 30°；位置与标定照片略有不同。',yaw=30,x=.62,y=.42),
    Pose('检查 · 上边靠近','上边靠近镜头，约 30°，放在画面偏左下。',pitch=-30,x=.38,y=.65),
    Pose('检查 · 左下角靠近','两个方向一起倾斜，让左下角最近。',yaw=-26,pitch=26,x=.6,y=.6),
)


def guide_step(role, accepted):
    """Successful counts index recommendations, not independently verified poses."""
    if role not in ('fit','validation') or type(accepted) is not int or accepted < 0:
        raise ValueError('Invalid guide role or accepted count')
    poses = FIT_POSES if role == 'fit' else VALIDATION_POSES
    return poses[min(accepted,len(poses)-1)], min(accepted+1,len(poses)), len(poses), accepted>=len(poses)


def project(pose, x, y):
    """Camera coordinates: x right, y down, z away. No optical-axis rotation."""
    yaw,pitch = math.radians(pose.yaw),math.radians(pose.pitch)
    yy,zz = y*math.cos(pitch),-y*math.sin(pitch)
    xx,z = x*math.cos(yaw)+zz*math.sin(yaw),-x*math.sin(yaw)+zz*math.cos(yaw)
    factor=210*pose.scale/(4+z)
    return (160+(pose.x-.5)*240+xx*factor,
            105+(pose.y-.5)*130+yy*factor)


def pose_svg(pose):
    """Trusted static SVG generated from constants; no devices, JS or user HTML."""
    def points(coords):
        return ' '.join(f'{x:.2f},{y:.2f}' for x,y in (project(pose,*p) for p in coords))
    title=escape(pose.title)
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 220" role="img" aria-label="{title}，蓝边靠近镜头。姿态示意，不是实时检测。">',
        f'<title>{title}</title>',
        '<rect x="8" y="16" width="304" height="171" rx="8" fill="#f6f8f5" stroke="#d7e2db"/>',
        '<path d="M160 22V181M14 102H306" stroke="#d7e2db" stroke-dasharray="4 5"/>']
    # Six-by-four symbolic checker pattern; marks are deliberately not real codes.
    for j in range(4):
        for i in range(6):
            x0,y0=-1.3+i*2.6/6,-.875+j*1.75/4
            x1,y1=x0+2.6/6,y0+1.75/4
            parts.append(f'<polygon points="{points([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])}" fill="{"#30483c" if (i+j)%2==0 else "white"}"/>')
            if (i+j)%2:
                dx,dy=(x1-x0)*.2,(y1-y0)*.2
                parts.append(f'<polygon points="{points([(x0+dx,y0+dy),(x1-dx,y0+dy),(x1-dx,y1-dy),(x0+dx,y1-dy)])}" fill="white" stroke="#637b6b" stroke-width="1.2"/>')
    border=[(-1.3,-.875),(1.3,-.875),(1.3,.875),(-1.3,.875)]
    parts.append(f'<polygon points="{points(border)}" fill="none" stroke="#30483c" stroke-width="1.5"/>')
    edges=[]
    if pose.yaw<0:edges.append([border[0],border[3]])
    if pose.yaw>0:edges.append([border[1],border[2]])
    if pose.pitch<0:edges.append([border[0],border[1]])
    if pose.pitch>0:edges.append([border[3],border[2]])
    for edge in edges:
        parts.append(f'<polyline points="{points(edge)}" stroke="#187bb5" stroke-width="5" stroke-linecap="round"/>')
    x,y=project(pose,0,-.875)
    parts.append(f'<text x="{x:.2f}" y="{y-9:.2f}" text-anchor="middle" fill="#43574b" font-size="18">上</text>')
    label='蓝边靠近镜头' if edges else '板面正对镜头'
    parts.append(f'<text x="160" y="207" text-anchor="middle" fill="{"#187bb5" if edges else "#43574b"}" font-size="18">{label}</text></svg>')
    return ''.join(parts)
