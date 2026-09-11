const models = [
  { name: 'YOLO11-N', status: 'ACTIVE', fps: '28 FPS', time: '36 ms', confidence: '96%', npu: '68%' },
  { name: 'SegFormer-B0', status: 'READY', fps: '22 FPS', time: '54 ms', confidence: '92%', npu: '52%' },
  { name: 'Depth Anything V2-S', status: 'ACTIVE', fps: '18 FPS', time: '66 ms', confidence: '89%', npu: '44%' },
  { name: 'ORB-SLAM3 / VIO', status: 'STANDBY', fps: '30 FPS', time: '18 ms', confidence: '94%', npu: '31%' },
]

function AiPerceptionPage() {
  return (
    <div className="space-y-5">
      <div>
        <div className="text-[12px] uppercase tracking-[0.28em] text-text/60">AI systems</div>
        <h1 className="mt-2 text-[32px] uppercase tracking-[0.14em] text-text">AI & PERCEPTION</h1>
      </div>

      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        {models.map((model) => (
          <div key={model.name} className="panel p-4">
            <div className="text-[10px] uppercase tracking-[0.2em] text-text/60">{model.name}</div>
            <div className="mt-3 inline-flex rounded-full border border-white/10 bg-white/5 px-2 py-1 text-[9px] uppercase tracking-[0.16em] text-text/80">{model.status}</div>
            <div className="mt-4 space-y-2 text-[10px] uppercase tracking-[0.14em] text-text/70">
              <div className="flex justify-between"><span>FPS</span><span className="text-text">{model.fps}</span></div>
              <div className="flex justify-between"><span>Inference</span><span className="text-text">{model.time}</span></div>
              <div className="flex justify-between"><span>Confidence</span><span className="text-text">{model.confidence}</span></div>
              <div className="flex justify-between"><span>NPU</span><span className="text-text">{model.npu}</span></div>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}

export default AiPerceptionPage
