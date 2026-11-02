"""Plot the archived low-duration follow-up and clock-only comparison."""
from pathlib import Path
import json
import sys
import zipfile
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from examples import gimbal_can as p


def main():
    font=Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus':False,'axes.spines.top':False,
        'axes.spines.right':False,'font.size':11,'figure.facecolor':'#f5f7fb',
        'axes.facecolor':'white','savefig.dpi':180,'text.color':'#0f172a'})
    out=ROOT/'docs/images/performance-20261102-followup'
    out.mkdir(exist_ok=True)
    with zipfile.ZipFile(ROOT/'docs/performance-20261102-followup/evidence.zip') as z:
        datasets={label:json.loads(z.read(f'current-diagnosis-{label}.json')) for label in ('legacy','highres')}
    summary=json.loads((ROOT/'docs/performance-20261102-followup/summary.json').read_text(encoding='utf-8'))
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    for ax,target,stat in zip(axes.flat,(.05,-.05,.1,-.1),summary['highres']):
        for index,run in enumerate(r for r in datasets['highres']['runs'] if r['target_a']==target):
            start=next(e['t'] for e in run['events'] if e['direction']=='TX' and bytes.fromhex(e['data'])[1]==1)
            data=[]
            for e in run['events']:
                if e['direction']=='RX' and e['id']==p.FAST_BASE+7:
                    f=p.decode_fast(e['id'],bytes.fromhex(e['data']),7)
                    data.append((e['t']-start,f['iq_a']*1000))
            ax.plot(*zip(*data),color=('#2563eb','#0891b2','#8b5cf6')[index],lw=1.15,label=f'重复 {index+1}')
        ax.axhline(target*1000,color='#f97316',ls='--',label='请求目标')
        ax.axvspan(max(abs(target)+.02,.11),.22,color='#10b981',alpha=.08)
        ax.set_xlim(-.01,.24)
        ax.set_title(f"目标 {target*1000:+.0f} mA · 三次平均 {stat['mean_iq_a']*1000:+.2f} mA",loc='left',fontweight='bold')
        ax.set_xlabel('距首条 IQ 指令 / s')
        ax.set_ylabel('Iq / mA')
        ax.grid(color='#e2e8f0')
    axes[0,0].legend(fontsize=8,loc='lower right')
    fig.suptitle('低时长电流跟随｜12 次实板短脉冲复测',x=.07,y=.98,ha='left',fontsize=21,fontweight='bold')
    fig.text(.07,.91,'2808 / 节点 7 · 每次 0.22 s · 原 PI / 原限压 / 原保护 · 更换为高分辨率主机调度时钟',color='#475569')
    fig.text(.07,.025,'保留原始波动；绿色为统计窗口。与旧 0.8 s 加速测试工况不同，不能据此宣称固件高速性能提升或持续输出能力。',fontsize=9,color='#64748b')
    fig.subplots_adjust(top=.81,bottom=.12,left=.08,right=.97,hspace=.45,wspace=.3)
    for ext in ('png','svg'):
        fig.savefig(out/f'short-current.{ext}')
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,6))
    for label,color,title in (('legacy','#94a3b8','原调度：15.625 ms 时钟分辨率'),('highres','#2563eb','修复后：高分辨率 QPC')):
        intervals=[]
        for run in datasets[label]['runs']:
            tx=[e['t'] for e in run['events'] if e['direction']=='TX' and bytes.fromhex(e['data'])[1]==1]
            intervals.extend((b-a)*1000 for a,b in zip(tx,tx[1:]))
        ax.hist(intervals,bins=25,alpha=.7,color=color,label=title)
    ax.axvline(10,color='#f97316',ls='--',label='目标周期 10 ms')
    ax.set_xlabel('相邻 IQ 主机发送间隔 / ms')
    ax.set_ylabel('间隔数量')
    ax.grid(color='#e2e8f0')
    ax.legend()
    fig.suptitle('主机刷新调度修复｜同为 0.22 s 短脉冲的时钟对照',x=.08,y=.98,ha='left',fontsize=19,fontweight='bold')
    fig.text(.08,.88,'每种时钟各 12 次测试；原始发送时间均由 perf_counter 记录',color='#475569')
    fig.text(.08,.025,'原间隔中位约 16.3 ms → 修复后约 10.2–10.5 ms。包含 Python / USB 开销，仍不属于硬实时调度。',fontsize=9,color='#64748b')
    fig.subplots_adjust(top=.78,bottom=.18,left=.1,right=.97)
    for ext in ('png','svg'):
        fig.savefig(out/f'clock-comparison.{ext}')
    plt.close(fig)


if __name__=='__main__':
    main()
