export function indexRoutePredictions(at,settings,sweptCollision){
  const times=new Map(),steps=new Map(),margins=new Map(),key=p=>`${p.x},${p.y}`;
  const positions=t=>{
    if(!times.has(t)){
      const points=at(t);times.set(t,{points,cells:new Set(points.map(key)),grid:points.every(p=>Number.isInteger(p.x)&&Number.isInteger(p.y))});
    }
    return times.get(t);
  };
  const transition=t=>{
    if(!steps.has(t)){
      const old=positions(t-1),next=positions(t);
      const grid=old.grid&&next.grid&&next.points.every((p,i)=>Math.abs(p.x-old.points[i].x)+Math.abs(p.y-old.points[i].y)<=1);
      steps.set(t,{old,next,grid,edges:grid?new Set(next.points.map((p,i)=>`${key(old.points[i])}>${key(p)}`)):null});
    }
    return steps.get(t);
  };
  return {
    canMove(a,b,t){
      const {old,next,grid,edges}=transition(t);
      // On a unit 4-neighbour integer grid, collisions are exactly shared
      // endpoints or opposing edge traversal. Use the original swept test for
      // other separation settings, fractional positions or nonunit predictions.
      if(settings.separation===1&&grid&&[a.x,a.y,b.x,b.y].every(Number.isInteger)&&Math.abs(a.x-b.x)+Math.abs(a.y-b.y)<=1)
        return !old.cells.has(key(a))&&!next.cells.has(key(b))&&!edges.has(`${key(b)}>${key(a)}`);
      return next.points.every((p,i)=>Math.hypot(b.x-p.x,b.y-p.y)>=settings.separation-1e-9&&!sweptCollision(a,b,old.points[i],p));
    },
    margin(p,t){
      if(!margins.has(t))margins.set(t,new Map());const cache=margins.get(t),cell=key(p);
      if(!cache.has(cell)){let result=5;for(const q of positions(t).points)result=Math.min(result,Math.hypot(p.x-q.x,p.y-q.y)-settings.separation);cache.set(cell,result);}
      return cache.get(cell);
    },
  };
}
