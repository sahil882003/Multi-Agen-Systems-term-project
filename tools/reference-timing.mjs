// Evaluation-only oracle. Runtime modules must never import this file.
// For nonnegative delays and positive quadratic costs, the componentwise
// earliest feasible point minimizes the objective. Difference constraints are
// relaxed to their least fixed point; positive cycles / upper excess fail.
export function referenceTiming(model) {
  const values=model.agents.map(a=>Array(a.length).fill(0));
  const dimensions=values.reduce((s,a)=>s+a.length,0);
  for(let round=0;round<=dimensions;round++) {
    let changed=false;
    model.agents.forEach((a,i)=>{
      for(let k=1;k<a.length;k++)if(values[i][k]<values[i][k-1]){values[i][k]=values[i][k-1];changed=true;}
    });
    for(const e of model.edges){const next=values[e.i][e.a]+e.gap;
      if(values[e.j][e.b]<next){values[e.j][e.b]=next;changed=true;}}
    if(values.some((v,i)=>v.some((x,k)=>x>model.agents[i].upper[k]+1e-9)))return {status:'infeasible'};
    if(!changed)return {status:'converged',values,objective:values.reduce((s,v)=>s+v.reduce((t,x)=>t+.5*x*x,0),0)};
  }
  return {status:'infeasible',reason:'Positive timing cycle'};
}
