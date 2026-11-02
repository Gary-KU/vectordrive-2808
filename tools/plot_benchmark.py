"""Render actual archived board telemetry; no synthetic or smoothed samples."""
import argparse
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from examples import gimbal_can as protocol


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--evidence', type=Path, default=ROOT/'docs/performance-20261102/evidence.zip')
    ap.add_argument('--output', type=Path, default=ROOT/'docs/images/performance-20261102')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    font = Path('C:/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus':False, 'font.size':11,
        'figure.facecolor':'#f5f7fb', 'axes.facecolor':'white', 'axes.spines.top':False,
        'axes.spines.right':False, 'axes.labelcolor':'#475569', 'text.color':'#0f172a',
        'axes.titleweight':'bold', 'axes.titlesize':13, 'grid.color':'#e2e8f0',
        'grid.alpha':.7, 'savefig.dpi':180})
    with zipfile.ZipFile(args.evidence) as z:
        can = json.loads(z.read('can.json'))
        current = json.loads(z.read('current.json'))

    def decoded(run):
        return [(e, protocol.decode_feedback(e['id'], bytes.fromhex(e['data']), current['node']))
                for e in run['events'] if e['direction']=='RX']

    def commands(run):
        return [e for e in run['events'] if e['direction']=='TX' and bytes.fromhex(e['data'])[1]==1]

    def style(ax, xlabel, ylabel):
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True)

    def save(fig, name, title, subtitle, footer):
        fig.suptitle(title, fontsize=21, fontweight='bold', x=.07, ha='left', y=.99)
        fig.text(.07,.90,subtitle,fontsize=11,color='#475569')
        fig.text(.07,.025,footer,fontsize=9,color='#64748b')
        bottom = .18 if fig.get_figheight() <= 6 else .12
        fig.subplots_adjust(top=.80,bottom=bottom,left=.08,right=.97,hspace=.45,wspace=.3)
        fig.savefig(args.output/(name+'.png'))
        fig.savefig(args.output/(name+'.svg'))
        plt.close(fig)

    def current_panel(ax, run):
        tx = commands(run)
        start = tx[0]['t']
        samples = [(e['t']-start, f['iq_a']*1000) for e,f in decoded(run) if f['type']=='fast']
        ax.plot(*zip(*samples), color='#2563eb',linewidth=1.15,label='实测 Iq（原始 FAST）')
        ax.axhline(run['target_a']*1000,color='#f97316',ls='--',linewidth=1.5,label='请求目标')
        ax.axvspan(.25,.75,color='#10b981',alpha=.08,label='统计窗口 250–750 ms')
        ax.set_xlim(-.03,.95)
        style(ax,'距首条 IQ 指令 / s','Iq / mA')
        # Match report validity filtering: only driver-enabled observations.
        steady=[f['iq_a']*1000 for e,f in decoded(run) if f['type']=='fast' and
                f['driver_enabled'] and .25<e['t']-start<.75]
        mean=np.mean(steady)
        rmse=np.sqrt(np.mean((np.array(steady)-run['target_a']*1000)**2))
        ax.set_title(f"目标 {run['target_a']*1000:+.0f} mA · 平均 {mean:+.1f} mA · RMSE {rmse:.1f} mA",loc='left')

    fig, axes=plt.subplots(2,2,figsize=(14,9))
    for ax,run in zip(axes.flat,current['runs'][:4]):
        current_panel(ax,run)
    axes[0,0].legend(fontsize=9,loc='lower right')
    save(fig,'current-tracking','正反向电流跟随｜原始实板数据',
         '2808 / 节点 7 · 每组约 0.8 s · 未平滑、未挑选采样点 · 2026-11-02',
         '空载加速时 ±100 mA 跟随下降；板内电流反馈未作外部仪表精度交叉标定。不能据此推定带载或持续电流能力。')

    rtt=[r['rtt_ms'] for r in can['replies']]
    fast=[e for e in can['events'] if e['direction']=='RX' and e['id']==protocol.FAST_BASE+can['node']]
    gaps=np.diff([e['adapter_timestamp'] for e in fast])*1000
    fig,axes=plt.subplots(1,2,figsize=(14,6))
    ax=axes[0]
    ax.hist(rtt,bins=32,color='#2563eb',edgecolor='white')
    ax.axvline(can['ack_rtt_ms']['median'],color='#f97316',ls='--',label='中位 2.76 ms')
    ax.axvline(can['ack_rtt_ms']['p95'],color='#10b981',ls='--',label='P95 3.65 ms')
    ax.set_title('592 / 592 条请求获成功 ACK',loc='left')
    style(ax,'主机发送 → 执行 ACK / ms','请求数')
    ax.legend()
    ax=axes[1]
    ax.plot([e['t'] for e in fast[1:]],gaps,color='#0891b2',lw=.6)
    ax.axhline(5,color='#f97316',ls='--',label='200 Hz 目标周期 5 ms')
    ax.set_title('FAST 实测 195.43 Hz · 最大间隔 5.97 ms',loc='left')
    style(ax,'测试时间 / s','相邻 FAST 帧间隔 / ms')
    ax.legend()
    save(fig,'can-performance','CAN 通信性能｜60 秒单板测试',
         'PCAN-USB / 1 Mbps · RTT 含 Windows 与 USB 开销；帧间隔使用 PCAN 时间戳',
         '累计 TX_LOSS 标志存在，严格通信验收未通过。592/592 查询成功不代表周期遥测零丢帧；FAST 无采样时间戳及序号。')

    run=current['runs'][4]
    last=commands(run)[-1]['t']
    statuses=[((e['t']-last)*1000,f) for e,f in decoded(run) if f['type']=='status']
    stop=next(x for x,f in statuses if x>=0 and f['command_timed_out'] and not f['driver_enabled'])
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    samples=[((e['t']-last)*1000,f['iq_a']*1000) for e,f in decoded(run) if f['type']=='fast']
    axes[0].plot(*zip(*samples),color='#2563eb',marker='.',ms=3,lw=1,label='实测 Iq')
    axes[0].set_title('停止续发 +50 mA 目标，继续监听反馈',loc='left')
    axes[0].legend(loc='upper left')
    axes[0].set_ylabel('Iq / mA')
    axes[1].step([x for x,f in statuses],[int(f['driver_enabled']) for x,f in statuses],where='post',
                 color='#2563eb',marker='.',label='状态反馈 driver_enabled')
    axes[1].step([x for x,f in statuses],[int(f['command_timed_out']) for x,f in statuses],where='post',
                 color='#f97316',marker='.',label='状态反馈 command_timed_out')
    axes[1].set_yticks([0,1],['0 / 否','1 / 是'])
    axes[1].set_ylim(-.15,1.25)
    axes[1].legend(loc='upper left',fontsize=9)
    axes[1].set_xlabel('距最后一条 IQ 主机发送 / ms')
    for ax in axes:
        ax.axvline(0,color='#64748b',ls=':',label='最后 IQ')
        ax.axvline(stop,color='#dc2626',ls='--')
        ax.set_xlim(-60,145)
        ax.grid(True)
    axes[0].annotate(f'{stop:.1f} ms：收到超时且驱动关闭状态',xy=(stop,0),xytext=(15,35),
                     arrowprops={'arrowstyle':'->','color':'#dc2626'},color='#dc2626',fontsize=10)
    save(fig,'timeout-stop','断流超时停机｜主机观察时间',
         '最后一条 IQ 后停止续发；未物理拔线 · 状态反馈为离散观察值',
         '约 105.8 ms 包含 CAN、USB 与状态发布延迟，不是示波器测得的 EN 引脚关断时间。结束重新 STOP 并收到成功执行 ACK。')

    fig,axes=plt.subplots(2,2,figsize=(14,9))
    current_panel(axes[0,0],current['runs'][0])
    current_panel(axes[0,1],current['runs'][1])
    axes[0,0].legend(fontsize=8,loc='lower right')
    axes[1,0].hist(rtt,bins=28,color='#2563eb',edgecolor='white')
    axes[1,0].set_title('CAN：592/592 ACK · 中位 2.76 ms / P95 3.65 ms',loc='left')
    style(axes[1,0],'主机执行 ACK 往返 / ms','请求数')
    axes[1,1].plot([e['t'] for e in fast[1:]],gaps,color='#0891b2',lw=.6)
    axes[1,1].set_title('FAST：195.4 Hz · 最大帧间隔 5.97 ms',loc='left')
    style(axes[1,1],'测试时间 / s','相邻 FAST 帧间隔 / ms')
    save(fig,'overview','VectorDrive 2808｜实板性能速览',
         'STM32F103 · 24 kHz PWM / 约 2 kHz 采样 · PCAN-USB · 2026-11-02',
         '本图为真实短时空载测试。累计 TX 丢弃标志存在；±100 mA 跟随下降。完整条件、限制与原始数据见性能报告。')
    print(args.output)


if __name__=='__main__':
    main()
