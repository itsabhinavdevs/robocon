import { useState, useEffect, useRef } from "react";

// ════════════════════════════════════════════════════════
// FIXED DATA (from rulebook — hardcoded, never changes)
// ════════════════════════════════════════════════════════
const H = {1:200,2:400,3:200,4:400,5:600,6:400,7:200,8:400,9:200,10:400,11:600,12:400};
const ADJ = {1:[2,4],2:[1,3,5],3:[2,6],4:[1,5,7],5:[2,4,6,8],6:[3,5,9],7:[4,8,10],8:[5,7,9,11],9:[6,8,12],10:[7,11],11:[8,10,12],12:[9,11]};
const MAX_CLIMB = 200;
const ENTRANCE = [1,2,3];
const EXITS    = [10,11,12];
const INF      = 1e9;
const GRC = {}; // graph row/col for heuristic
for(let b=1;b<=12;b++) GRC[b]={r:Math.floor((b-1)/3), c:(b-1)%3};
const GRID_ROWS = [[10,11,12],[7,8,9],[4,5,6],[1,2,3]]; // top=exit, bottom=entrance

// ════════════════════════════════════════════════════════
// CORE ALGORITHM FUNCTIONS
// ════════════════════════════════════════════════════════
const manhat = (a,b) => Math.abs(GRC[a].r-GRC[b].r)+Math.abs(GRC[a].c-GRC[b].c);

function edgeCost(a, b, blocked) {
  if(blocked.has(b)) return INF;
  const d = Math.abs(H[a]-H[b]);
  if(d > MAX_CLIMB) return INF;
  return 1 + (d/200)*0.5; // base=1, height penalty up to 0.5
}

function astar(start, goal, blocked) {
  if(start===goal) return {cost:0, path:[start]};
  const g={}, pr={}, open=new Set([start]);
  for(let i=1;i<=12;i++) g[i]=INF;
  g[start]=0;
  while(open.size){
    let cur=null,bf=INF;
    for(const n of open){const f=g[n]+manhat(n,goal);if(f<bf){bf=f;cur=n;}}
    open.delete(cur);
    if(cur===goal){
      const path=[]; let c=goal;
      while(c!==undefined){path.unshift(c);c=pr[c];}
      return {cost:g[goal], path};
    }
    for(const nb of ADJ[cur]){
      const ec=edgeCost(cur,nb,blocked);
      if(ec===INF) continue;
      const ng=g[cur]+ec;
      if(ng<g[nb]){g[nb]=ng;pr[nb]=cur;open.add(nb);}
    }
  }
  return {cost:INF, path:[]};
}

function allPairs(blocked){
  const C={},P={};
  for(let a=1;a<=12;a++) for(let b=1;b<=12;b++){
    const r=a===b?{cost:0,path:[a]}:astar(a,b,blocked);
    C[`${a}~${b}`]=r.cost; P[`${a}~${b}`]=r.path;
  }
  return {C,P};
}

function perms(arr){
  if(arr.length<=1) return [[...arr]];
  return arr.flatMap((_,i)=>perms([...arr.slice(0,i),...arr.slice(i+1)]).map(p=>[arr[i],...p]));
}

function getFacing(from,to){
  const dr=GRC[to].r-GRC[from].r, dc=GRC[to].c-GRC[from].c;
  if(dr>0)return'NORTH'; if(dr<0)return'SOUTH'; if(dc>0)return'EAST'; return'WEST';
}

// ════════════════════════════════════════════════════════
// MAIN PLANNER — returns logs[] and result{}
// ════════════════════════════════════════════════════════
function runPlanning(blockStates){
  const logs=[], push=(t,m)=>logs.push({t,m});
  const r2=[],fk=[];
  for(let b=1;b<=12;b++){
    if(blockStates[b]==='r2kfs') r2.push(b);
    if(blockStates[b]==='fake') fk.push(b);
  }
  const blocked=new Set(fk);

  push('h','╔══════════════════════════════════════════╗');
  push('h','║   MEIHUA FOREST PATH PLANNER  v2.0      ║');
  push('h','╚══════════════════════════════════════════╝');
  push('sep','');
  push('ph','[ PHASE 0 ]  LED screen matrix decode');
  push('i',`  R2 KFS on blocks   → [ ${r2.join(', ')} ]`);
  push('i',`  Fake KFS on blocks → [ ${fk.join(', ')||'none'} ]`);
  if(r2.length!==4){push('e',`  ✗ FAIL: need 4 R2 KFS, got ${r2.length}`);return{logs,result:null};}
  push('ok','  ✓ Validation OK — 4 R2 KFS confirmed');
  push('sep','');

  push('ph','[ PHASE 1 ]  Graph construction');
  push('d','  Heights (fixed, rulebook §Meihua Forest):');
  push('d','  Row 1 (entrance): B1=200  B2=400  B3=200');
  push('d','  Row 2:            B4=400  B5=600  B6=400');
  push('d','  Row 3:            B7=200  B8=400  B9=200');
  push('d','  Row 4 (exit):     B10=400 B11=600 B12=400');
  push('i',`  Max climb constraint: ${MAX_CLIMB}mm per step`);
  const bad=[];
  for(let a=1;a<=12;a++) for(const b of ADJ[a]) if(a<b&&Math.abs(H[a]-H[b])>MAX_CLIMB) bad.push(`B${a}↔B${b}`);
  push('i',`  Height-blocked edges: ${bad.length?bad.join(', '):'none (all steps ≤200mm)'}`);
  push('i',`  Rule-blocked blocks (fake KFS): ${fk.map(b=>`B${b}`).join(', ')||'none'}`);
  push('sep','');

  push('ph','[ PHASE 2 ]  All-pairs A* (12×12 = 144 pairs)');
  const {C,P}=allPairs(blocked);
  const reach=Object.values(C).filter(v=>v<INF).length;
  push('ok',`  ✓ Done — ${reach}/144 pairs reachable`);
  push('d',`  B1→B12: cost=${C['1~12']<INF?C['1~12'].toFixed(2):'∞'}  path=[${(P['1~12']||[]).join('→')}]`);
  push('d',`  B3→B10: cost=${C['3~10']<INF?C['3~10'].toFixed(2):'∞'}  path=[${(P['3~10']||[]).join('→')}]`);
  push('d',`  B2→B11: cost=${C['2~11']<INF?C['2~11'].toFixed(2):'∞'}  path=[${(P['2~11']||[]).join('→')}]`);
  push('sep','');

  push('ph','[ PHASE 3 ]  TSP — optimal visit order');
  const eKFS=r2.filter(k=>ENTRANCE.includes(k));
  push('i',`  KFS targets: ${r2.map(b=>`B${b}(${H[b]}mm)`).join('  ')}`);
  push('c',`  Rule 4.4.14: KFS in entrance zone = [${eKFS.map(b=>`B${b}`).join(',')||'none'}]`);

  let ps=perms(r2);
  if(eKFS.length){
    ps=ps.filter(p=>eKFS.includes(p[0]));
    push('c',`  → Filtered to ${ps.length} valid permutations (entrance-first enforced)`);
  } else push('i',`  → No entrance constraint — evaluating all ${ps.length} permutations`);
  push('i',`  → Testing ${ps.length} orders × ${ENTRANCE.length} entry blocks = ${ps.length*ENTRANCE.length} combinations`);
  push('sep','');

  let best={cost:INF,perm:null,plan:null,entry:null,exit:null};

  for(const perm of ps){
    for(const en of ENTRANCE){
      let tc=0, cur=en, plan=[], ok=true;
      for(const kb of perm){
        const adj2=ADJ[kb].filter(n=>!blocked.has(n));
        if(!adj2.length){ok=false;break;}
        let bc=INF,bp=null;
        for(const a of adj2){if(C[`${cur}~${a}`]<bc){bc=C[`${cur}~${a}`];bp=a;}}
        if(bc===INF){ok=false;break;}
        plan.push({type:'move',from:cur,to:bp,path:[...P[`${cur}~${bp}`]],cost:bc});
        plan.push({type:'pickup',stand:bp,kfs:kb,face:getFacing(bp,kb)});
        tc+=bc; cur=bp;
      }
      if(!ok) continue;
      let ec=INF,eb=null;
      for(const ex of EXITS){if(C[`${cur}~${ex}`]<ec){ec=C[`${cur}~${ex}`];eb=ex;}}
      if(ec===INF) continue;
      plan.push({type:'exit',from:cur,to:eb,path:[...P[`${cur}~${eb}`]],cost:ec});
      tc+=ec;
      const nb=tc<best.cost;
      push(nb?'best':'pm',`  [${perm.map(b=>`B${b}`).join('→')}] en=B${en} ex=B${eb}  cost=${tc.toFixed(2)}${nb?' ← NEW BEST':''}`);
      if(nb) best={cost:tc,perm:[...perm],plan:[...plan],entry:en,exit:eb};
    }
  }

  if(!best.plan){push('e','  ✗ No valid path found! Check KFS layout.');return{logs,result:null};}

  push('sep','');
  push('ok','╔══════════════════════════════════════╗');
  push('ok',`║  OPTIMAL SOLUTION                    ║`);
  push('ok',`║  Order : B${best.entry}→${best.perm.map(b=>`B${b}`).join('→')}     ║`);
  push('ok',`║  Entry : B${best.entry}   Exit : B${best.exit}            ║`);
  push('ok',`║  Cost  : ${best.cost.toFixed(2)} units               ║`);
  push('ok','╚══════════════════════════════════════╝');
  push('sep','');

  push('ph','[ PHASE 4 ]  Execution plan');
  best.plan.forEach((s,i)=>{
    if(s.type==='move')   push('st',`  #${i+1} MOVE    [${s.path.join('→')}]  cost=${s.cost.toFixed(2)}`);
    else if(s.type==='pickup') push('pk',`  #${i+1} PICKUP  stand=B${s.stand}  kfs=B${s.kfs}  face=${s.face}`);
    else                  push('ex',`  #${i+1} EXIT    [${s.path.join('→')}]  → B${s.to}`);
  });

  return {logs, result:{...best}};
}

// ════════════════════════════════════════════════════════
// EXECUTION FRAMES — drives the animation
// ════════════════════════════════════════════════════════
const rnd=(a,b)=>Math.floor(Math.random()*(b-a+1)+a);

function buildFrames(plan, entry){
  const frames=[];
  let path=[entry], picked=[];

  frames.push({robot:entry,path:[...path],action:'ENTER',picked:[...picked],
    msg:`Robot enters forest at B${entry} (${H[entry]}mm)`,
    sensor:{odo:`X:0mm  Y:0mm  θ:0.0°`,imu:`pitch:0.0°  roll:0.0°`,cam:`Block B${entry} edges locked in depth cam`,fix:`Odometry zeroed at B${entry} block center`}
  });

  for(const s of plan){
    if(s.type==='move'||s.type==='exit'){
      for(let i=1;i<s.path.length;i++){
        const f=s.path[i-1], t=s.path[i];
        const hd=H[t]-H[f];
        const xe=rnd(-22,22), ye=rnd(-18,18);
        path=[...path,t];
        frames.push({robot:t,path:[...path],action:'MOVE',picked:[...picked],
          msg:`${s.type==='exit'?'Exiting:':'Moving:'} B${f}(${H[f]}mm) → B${t}(${H[t]}mm)  Δh=${hd>0?'+':''}${hd}mm`,
          sensor:{
            odo:`X:${(GRC[t].c-GRC[entry].c)*1200}mm  Y:${(GRC[t].r-GRC[entry].r)*1200}mm`,
            imu:`pitch:${hd>0?'+':''}${(hd/200*2.8).toFixed(1)}°  roll:0.${rnd(0,4)}°`,
            cam:`B${t} edge detected — offset ΔX=${xe}mm  ΔY=${ye}mm`,
            fix:`Correction applied: ΔX=${-xe}mm  ΔY=${-ye}mm  → locked to B${t} center`
          }
        });
      }
    } else if(s.type==='pickup'){
      const cf=(0.87+Math.random()*0.11).toFixed(2);
      frames.push({robot:s.stand,path:[...path],action:'SCAN',picked:[...picked],scanTarget:s.kfs,
        msg:`Camera scan: KFS on B${s.kfs}  (standing B${s.stand}, facing ${s.face})`,
        sensor:{odo:`stable`,imu:`pitch:0.1°  roll:0.1°`,cam:`YOLOv8-Nano: class=R2_KFS  conf=${cf}  bbox[234,198,312,276]`,fix:`Confirmed real KFS (not fake) — proceed with pickup`}
      });
      picked=[...picked,s.kfs];
      frames.push({robot:s.stand,path:[...path],action:'PICKUP',picked:[...picked],
        msg:`KFS picked from B${s.kfs} — total ${picked.length}/4`,
        sensor:{odo:`stable`,imu:`pitch:0.0°  roll:0.0°`,cam:`Gripper cam: KFS secured in holder`,fix:`Limit switch CLOSED  |  Grip force: ${rnd(9,14)}N  |  KFS locked`}
      });
    }
  }

  frames.push({robot:frames.at(-1).robot,path:[...path],action:'DONE',picked:[...picked],
    msg:`Forest complete — all 4 KFS collected. Heading to ramp.`,
    sensor:{odo:`Exit block reached`,imu:`pitch rising — ramp edge detected`,cam:`Ramp structure in depth cam FOV`,fix:`Region FSM: MEIHUA_FOREST → ON_RAMP`}
  });
  return frames;
}

// ════════════════════════════════════════════════════════
// LOG COLOR MAP
// ════════════════════════════════════════════════════════
const LC = {
  h:'var(--color-text-info)',
  ph:'var(--color-text-primary)',
  i:'var(--color-text-secondary)',
  d:'var(--color-text-tertiary)',
  ok:'var(--color-text-success)',
  e:'var(--color-text-danger)',
  c:'var(--color-text-warning)',
  best:'#d97706',
  pm:'var(--color-text-tertiary)',
  st:'var(--color-text-info)',
  pk:'var(--color-text-success)',
  ex:'var(--color-text-info)',
  sep:'var(--color-border-tertiary)',
};

// ════════════════════════════════════════════════════════
// MAIN COMPONENT
// ════════════════════════════════════════════════════════
export default function ForestPlanner() {
  const [blocks, setBlocks] = useState(()=>{const s={};for(let i=1;i<=12;i++)s[i]='empty';return s;});
  const [phase,  setPhase]  = useState('setup');
  const [logs,   setLogs]   = useState([]);
  const [frames, setFrames] = useState([]);
  const [fi,     setFi]     = useState(0);
  const [playing,setPlaying]= useState(false);
  const [speed,  setSpeed]  = useState(1100);
  const [result, setResult] = useState(null);
  const logRef = useRef(null);
  const tmr    = useRef(null);

  const kfsCnt = Object.values(blocks).filter(s=>s==='r2kfs').length;
  const fkCnt  = Object.values(blocks).filter(s=>s==='fake').length;
  const frame  = frames[fi];

  // Auto-scroll log
  useEffect(()=>{if(logRef.current) logRef.current.scrollTop=logRef.current.scrollHeight;},[logs]);

  // Animation timer
  useEffect(()=>{
    if(playing && phase==='executing'){
      tmr.current=setInterval(()=>{
        setFi(p=>{
          if(p>=frames.length-1){setPlaying(false);setPhase('done');clearInterval(tmr.current);return p;}
          return p+1;
        });
      },speed);
    } else clearInterval(tmr.current);
    return()=>clearInterval(tmr.current);
  },[playing,phase,speed,frames.length]);

  const clickBlock=(id)=>{
    if(phase!=='setup') return;
    setBlocks(prev=>{
      const cur=prev[id];
      const kc=Object.values(prev).filter(s=>s==='r2kfs').length;
      const fc=Object.values(prev).filter(s=>s==='fake').length;
      let next='empty';
      if(cur==='empty')    next=kc<4?'r2kfs':fc<1?'fake':'empty';
      else if(cur==='r2kfs') next=fc<1?'fake':'empty';
      else                 next='empty';
      return {...prev,[id]:next};
    });
  };

  const startPlan=()=>{
    if(kfsCnt!==4){alert('Place exactly 4 R2 KFS (green) first!');return;}
    setPhase('computing');
    setTimeout(()=>{
      const {logs:L,result:R}=runPlanning(blocks);
      setLogs(L);
      if(R){
        const F=buildFrames(R.plan,R.entry);
        setFrames(F);setResult(R);setFi(0);setPhase('executing');setPlaying(true);
      } else setPhase('setup');
    },350);
  };

  const reset=()=>{
    clearInterval(tmr.current);
    setBlocks(()=>{const s={};for(let i=1;i<=12;i++)s[i]='empty';return s;});
    setPhase('setup');setLogs([]);setFrames([]);setFi(0);setPlaying(false);setResult(null);
  };

  const getVis=(id)=>{
    if(!frame) return blocks[id];
    if(frame.robot===id)              return 'robot';
    if(frame.scanTarget===id)         return 'scan';
    if(frame.picked?.includes(id))    return 'picked';
    if(frame.path?.includes(id))      return 'pathed';
    return blocks[id];
  };

  // Block visual config
  const BV={
    empty:  {bg:'var(--color-background-secondary)',border:'var(--color-border-tertiary)',accent:''},
    r2kfs:  {bg:'var(--color-background-success)',  border:'var(--color-border-success)', accent:'var(--color-text-success)'},
    fake:   {bg:'var(--color-background-danger)',   border:'var(--color-border-danger)',  accent:'var(--color-text-danger)'},
    robot:  {bg:'var(--color-background-info)',     border:'var(--color-border-info)',    accent:'var(--color-text-info)'},
    scan:   {bg:'var(--color-background-warning)',  border:'var(--color-border-warning)', accent:'var(--color-text-warning)'},
    picked: {bg:'var(--color-background-success)',  border:'var(--color-border-success)', accent:'var(--color-text-success)'},
    pathed: {bg:'var(--color-background-info)',     border:'var(--color-border-info)',    accent:'var(--color-text-info)'},
  };

  const actionColor={MOVE:'var(--color-text-info)',ENTER:'var(--color-text-secondary)',SCAN:'var(--color-text-warning)',PICKUP:'var(--color-text-success)',DONE:'var(--color-text-success)',EXIT:'var(--color-text-secondary)'};

  const S={
    wrap:{fontFamily:'var(--font-mono)',fontSize:13,color:'var(--color-text-primary)',padding:'12px 0'},
    row:{display:'grid',gridTemplateColumns:'300px 1fr',gap:12,alignItems:'start'},
    panel:{background:'var(--color-background-secondary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-lg)',padding:'12px',marginBottom:10},
    panelTitle:{fontSize:11,fontWeight:500,color:'var(--color-text-tertiary)',letterSpacing:'0.08em',textTransform:'uppercase',marginBottom:8,paddingBottom:6,borderBottom:'0.5px solid var(--color-border-tertiary)'},
    logBox:{height:260,overflowY:'auto',fontFamily:'var(--font-mono)',fontSize:11.5,lineHeight:1.7,background:'var(--color-background-primary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-md)',padding:'8px 10px'},
    sensorGrid:{display:'grid',gridTemplateColumns:'1fr 1fr',gap:8},
    sensorCard:{background:'var(--color-background-primary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-md)',padding:'7px 9px'},
    sensorLabel:{fontSize:10,fontWeight:500,color:'var(--color-text-tertiary)',marginBottom:3},
    sensorVal:{fontSize:12,color:'var(--color-text-secondary)',fontFamily:'var(--font-mono)',wordBreak:'break-word'},
    btn:{padding:'5px 12px',fontSize:12,border:'0.5px solid var(--color-border-secondary)',borderRadius:'var(--border-radius-md)',background:'transparent',color:'var(--color-text-secondary)',cursor:'pointer',fontFamily:'var(--font-mono)'},
    btnP:{padding:'6px 16px',fontSize:12,border:'0.5px solid var(--color-border-success)',borderRadius:'var(--border-radius-md)',background:'var(--color-background-success)',color:'var(--color-text-success)',cursor:'pointer',fontFamily:'var(--font-mono)',fontWeight:500},
    hint:{fontSize:11,color:'var(--color-text-tertiary)',lineHeight:1.5,padding:'6px 8px',background:'var(--color-background-primary)',borderRadius:'var(--border-radius-md)',borderLeft:'2px solid var(--color-border-secondary)',marginBottom:8},
  };

  return (
    <div style={S.wrap}>
      {/* ── HEADER ── */}
      <div style={{display:'flex',alignItems:'center',justifyContent:'space-between',marginBottom:12,paddingBottom:10,borderBottom:'0.5px solid var(--color-border-tertiary)'}}>
        <div>
          <div style={{fontSize:15,fontWeight:500,color:'var(--color-text-primary)',letterSpacing:'0.04em'}}>Meihua forest path planner</div>
          <div style={{fontSize:11,color:'var(--color-text-tertiary)'}}>Optimal TSP-A* navigation — R2 autonomous system</div>
        </div>
        <div style={{display:'flex',gap:8,alignItems:'center'}}>
          <span style={{fontSize:11,color:'var(--color-text-tertiary)'}}>Region: MEIHUA_FOREST</span>
          <span style={{fontSize:11,padding:'2px 10px',borderRadius:'var(--border-radius-md)',
            background:phase==='executing'?'var(--color-background-success)':phase==='done'?'var(--color-background-success)':phase==='computing'?'var(--color-background-warning)':'var(--color-background-secondary)',
            color:phase==='executing'||phase==='done'?'var(--color-text-success)':phase==='computing'?'var(--color-text-warning)':'var(--color-text-secondary)',
            border:'0.5px solid var(--color-border-secondary)'
          }}>{phase}</span>
        </div>
      </div>

      {/* ── MAIN LAYOUT ── */}
      <div style={S.row}>

        {/* ── LEFT: GRID + CONTROLS ── */}
        <div>
          {/* Forest grid */}
          <div style={S.panel}>
            <div style={S.panelTitle}>Forest grid — LED screen input</div>
            <div style={{fontSize:10,color:'var(--color-text-tertiary)',textAlign:'center',marginBottom:6}}>↑ exit → arena  (blocks 10–12)</div>
            <div style={{display:'flex',flexDirection:'column',gap:4}}>
              {GRID_ROWS.map((row,ri)=>(
                <div key={ri} style={{display:'grid',gridTemplateColumns:'repeat(3,1fr)',gap:4}}>
                  {row.map(id=>{
                    const vis=getVis(id);
                    const bv=BV[vis]||BV.empty;
                    const isKfs=blocks[id]==='r2kfs', isFake=blocks[id]==='fake';
                    const isPicked=frame?.picked?.includes(id)&&isKfs;
                    return (
                      <div key={id} onClick={()=>clickBlock(id)} style={{
                        background:bv.bg, border:`0.5px solid ${bv.border}`,
                        borderRadius:'var(--border-radius-md)', padding:'7px 4px',
                        cursor:phase==='setup'?'pointer':'default',
                        position:'relative', minHeight:62,
                        display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'center',
                        transition:'background 0.15s,border-color 0.15s',
                        userSelect:'none',
                      }}>
                        {/* Height bar */}
                        <div style={{position:'absolute',bottom:0,left:0,right:0,height:`${(H[id]/600)*35}%`,
                          background:bv.accent||'var(--color-border-tertiary)',opacity:0.12,borderRadius:'0 0 var(--border-radius-md) var(--border-radius-md)'}}/>
                        <div style={{fontSize:10,fontWeight:500,color:'var(--color-text-tertiary)',lineHeight:1}}>B{id}</div>
                        <div style={{fontSize:11,fontWeight:500,color:bv.accent||'var(--color-text-secondary)',lineHeight:1.4}}>
                          {H[id]}<span style={{fontSize:9}}>mm</span>
                        </div>
                        {/* State icons (using text, not emoji) */}
                        {isKfs && !isPicked && vis!=='robot' && vis!=='scan' &&
                          <div style={{fontSize:12,color:'var(--color-text-success)',lineHeight:1}}>◆</div>}
                        {isFake && <div style={{fontSize:12,color:'var(--color-text-danger)',lineHeight:1}}>✕</div>}
                        {vis==='robot' && <div style={{width:14,height:14,borderRadius:'50%',background:'var(--color-text-info)',margin:'2px auto 0'}}/>}
                        {vis==='scan'  && <div style={{fontSize:10,color:'var(--color-text-warning)',lineHeight:1,fontWeight:500}}>CAM</div>}
                        {isPicked      && <div style={{fontSize:9,color:'var(--color-text-success)',fontWeight:500}}>PICKED</div>}
                      </div>
                    );
                  })}
                </div>
              ))}
            </div>
            <div style={{fontSize:10,color:'var(--color-text-tertiary)',textAlign:'center',marginTop:6}}>↓ entrance ← martial club  (blocks 1–3)</div>
            {/* Legend */}
            <div style={{display:'flex',flexWrap:'wrap',gap:10,marginTop:8,paddingTop:6,borderTop:'0.5px solid var(--color-border-tertiary)'}}>
              {[['◆','var(--color-text-success)','R2 KFS'],['✕','var(--color-text-danger)','Fake KFS'],
                ['●','var(--color-text-info)','Robot pos'],['▪','var(--color-text-info)','Path'],
                ['◆','var(--color-text-success)','Picked']].map(([s,c,l],i)=>(
                <div key={i} style={{display:'flex',alignItems:'center',gap:4}}>
                  <span style={{color:c,fontSize:i===2?10:12}}>{s}</span>
                  <span style={{fontSize:10,color:'var(--color-text-tertiary)'}}>{l}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Controls */}
          <div style={S.panel}>
            <div style={S.panelTitle}>Controls</div>
            {/* Counters */}
            <div style={{display:'grid',gridTemplateColumns:'repeat(3,1fr)',gap:8,marginBottom:10}}>
              {[
                [`${kfsCnt}/4`,'R2 KFS',kfsCnt===4?'var(--color-text-success)':'var(--color-text-warning)'],
                [`${fkCnt}/1`,'Fake',   'var(--color-text-danger)'],
                [result?result.cost.toFixed(1):'—','Opt cost','var(--color-text-info)'],
              ].map(([v,l,c])=>(
                <div key={l} style={{background:'var(--color-background-primary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-md)',padding:'7px 4px',textAlign:'center'}}>
                  <div style={{fontSize:16,fontWeight:500,color:c}}>{v}</div>
                  <div style={{fontSize:9,color:'var(--color-text-tertiary)',letterSpacing:'0.06em'}}>{l}</div>
                </div>
              ))}
            </div>

            {phase==='setup' && <div style={S.hint}>
              Click any block to cycle: empty → R2 KFS (◆ green) → Fake (✕ red) → empty.<br/>
              Place exactly 4 R2 KFS, then press Plan.
            </div>}

            <div style={{display:'flex',flexWrap:'wrap',gap:6,marginBottom:8}}>
              {phase==='setup' && (
                <button onClick={startPlan} style={{...S.btnP, opacity:kfsCnt===4?1:0.5, cursor:kfsCnt===4?'pointer':'not-allowed'}}>
                  Plan + execute
                </button>
              )}
              {phase==='executing' && <>
                <button onClick={()=>setPlaying(!playing)} style={S.btn}>{playing?'Pause':'Play'}</button>
                <button onClick={()=>setFi(p=>Math.max(0,p-1))} style={S.btn}>◀ back</button>
                <button onClick={()=>setFi(p=>Math.min(frames.length-1,p+1))} style={S.btn}>next ▶</button>
              </>}
              {phase==='done' && (
                <button onClick={()=>{setFi(0);setPhase('executing');setPlaying(true);}} style={S.btn}>Replay</button>
              )}
              <button onClick={reset} style={{...S.btn,color:'var(--color-text-danger)',borderColor:'var(--color-border-danger)'}}>Reset</button>
            </div>

            {(phase==='executing'||phase==='done') && (
              <div style={{display:'flex',gap:6,alignItems:'center'}}>
                <span style={{fontSize:10,color:'var(--color-text-tertiary)'}}>Speed:</span>
                {[['fast',500],['normal',1100],['slow',2000]].map(([l,v])=>(
                  <button key={l} onClick={()=>setSpeed(v)} style={{
                    padding:'2px 8px',fontSize:10,cursor:'pointer',
                    border:`0.5px solid ${speed===v?'var(--color-border-info)':'var(--color-border-tertiary)'}`,
                    background:speed===v?'var(--color-background-info)':'transparent',
                    color:speed===v?'var(--color-text-info)':'var(--color-text-tertiary)',
                    borderRadius:'var(--border-radius-md)',fontFamily:'var(--font-mono)',
                  }}>{l}</button>
                ))}
              </div>
            )}
            {phase==='computing' && <div style={{color:'var(--color-text-warning)',fontSize:12,marginTop:8}}>Computing optimal path...</div>}
          </div>

          {/* Current action */}
          {frame && (
            <div style={{...S.panel, borderColor:frame.action==='PICKUP'||frame.action==='DONE'?'var(--color-border-success)':'var(--color-border-tertiary)'}}>
              <div style={S.panelTitle}>Current action</div>
              <div style={{fontSize:13,color:actionColor[frame.action]||'var(--color-text-primary)',lineHeight:1.5,marginBottom:8,fontWeight:500}}>
                {frame.msg}
              </div>
              {/* KFS progress dots */}
              <div style={{display:'flex',gap:6,alignItems:'center',marginBottom:4}}>
                {[0,1,2,3].map(i=>(
                  <div key={i} style={{width:20,height:20,borderRadius:'var(--border-radius-md)',
                    background:(frame.picked||[]).length>i?'var(--color-background-success)':'var(--color-background-primary)',
                    border:`0.5px solid ${(frame.picked||[]).length>i?'var(--color-border-success)':'var(--color-border-tertiary)'}`,
                    display:'flex',alignItems:'center',justifyContent:'center',fontSize:9,
                    color:'var(--color-text-success)',
                  }}>{(frame.picked||[]).length>i?'◆':''}</div>
                ))}
                <span style={{fontSize:10,color:'var(--color-text-tertiary)',marginLeft:4}}>KFS {(frame.picked||[]).length}/4</span>
                <span style={{fontSize:10,color:'var(--color-text-tertiary)',marginLeft:'auto'}}>frame {fi+1}/{frames.length}</span>
              </div>
              {/* Progress bar */}
              <div style={{height:3,background:'var(--color-border-tertiary)',borderRadius:2,overflow:'hidden'}}>
                <div style={{height:'100%',width:`${((fi+1)/frames.length)*100}%`,background:'var(--color-text-info)',transition:'width 0.3s'}}/>
              </div>
            </div>
          )}
        </div>

        {/* ── RIGHT: LOG + SENSOR + SUMMARY ── */}
        <div>
          {/* Algorithm log */}
          <div style={S.panel}>
            <div style={S.panelTitle}>Algorithm log — full decision trace</div>
            <div ref={logRef} style={S.logBox}>
              {logs.length===0 && (
                <div style={{color:'var(--color-text-tertiary)',padding:'20px 0',textAlign:'center',lineHeight:2}}>
                  [awaiting input]<br/>
                  <span style={{fontSize:10}}>Place 4 R2 KFS on the grid and press Plan to see the full algorithm trace</span>
                </div>
              )}
              {logs.map((l,i)=>(
                <div key={i} style={{color:LC[l.t]||'var(--color-text-secondary)',whiteSpace:'pre',letterSpacing:'0.01em'}}>
                  {l.m}
                </div>
              ))}
            </div>
          </div>

          {/* Sensor data */}
          {frame && (
            <div style={S.panel}>
              <div style={S.panelTitle}>Live sensor data — correction pipeline</div>
              <div style={S.sensorGrid}>
                {[
                  ['Odometry',    frame.sensor.odo, 'var(--color-text-info)'],
                  ['IMU / gyro',  frame.sensor.imu, 'var(--color-text-secondary)'],
                  ['Depth camera',frame.sensor.cam, 'var(--color-text-info)'],
                  ['Correction applied', frame.sensor.fix, 'var(--color-text-success)'],
                ].map(([lbl,val,c])=>(
                  <div key={lbl} style={{...S.sensorCard, borderLeft:`2px solid ${c}`}}>
                    <div style={{...S.sensorLabel, color:c}}>{lbl}</div>
                    <div style={S.sensorVal}>{val}</div>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Path summary */}
          {result && (
            <div style={S.panel}>
              <div style={S.panelTitle}>Optimal path summary</div>
              <div style={{display:'grid',gridTemplateColumns:'repeat(3,1fr)',gap:8,marginBottom:10}}>
                {[
                  ['Total cost', result.cost.toFixed(2)+' u', 'var(--color-text-info)'],
                  ['Blocks in path', (frame?.path?.length||'—')+' steps', 'var(--color-text-secondary)'],
                  ['Entry block', `B${result.entry} (${H[result.entry]}mm)`, 'var(--color-text-secondary)'],
                ].map(([l,v,c])=>(
                  <div key={l} style={{background:'var(--color-background-primary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-md)',padding:'7px 8px'}}>
                    <div style={{fontSize:9,color:'var(--color-text-tertiary)',marginBottom:3}}>{l}</div>
                    <div style={{fontSize:13,color:c,fontFamily:'var(--font-mono)',fontWeight:500}}>{v}</div>
                  </div>
                ))}
              </div>
              <div style={{background:'var(--color-background-primary)',border:'0.5px solid var(--color-border-tertiary)',borderRadius:'var(--border-radius-md)',padding:'8px 10px'}}>
                <div style={{fontSize:9,color:'var(--color-text-tertiary)',marginBottom:4}}>Optimal visit order</div>
                <div style={{fontSize:13,color:'var(--color-text-success)',fontFamily:'var(--font-mono)',letterSpacing:'0.05em'}}>
                  B{result.entry} → {result.perm.map(b=>`B${b}`).join(' → ')} → exit B{result.exit}
                </div>
              </div>
              <div style={{marginTop:8,fontSize:11,color:'var(--color-text-tertiary)',lineHeight:1.6}}>
                Cost = sum of A* path costs between consecutive pickup positions.
                Edge cost = 1.0 + (Δheight/200)×0.5. Fake KFS blocks = cost ∞.
                All {(function(){let p=perms(result.perm);return p.length;})()} permutations of KFS visit order were evaluated across {ENTRANCE.length} entry blocks.
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
