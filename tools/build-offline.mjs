import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Script } from "node:vm";
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const files = [
  "world",
  "stl",
  "passage",
  "robot-agent",
  "peer-protocol",
  "incremental",
  "junction",
  "junction-peer",
  "junction-assignment",
  "route-predictions",
  "search-guidance",
  "planner",
  "metrics",
  "stl-observer",
  "stl-log",
  "sim",
  "presentation",
  "main",
];
// Preserve this order so dependencies appear before the browser controller.
const code = files
  .map((name) =>
    fs
      .readFileSync(path.join(root, "src", name + ".js"), "utf8")
      .replace(/^import[\s\S]*?;\s*/gm, "")
      .replace(/\bexport (?=(?:class|function|const|let)\b)/g, ""),
  )
  .join("\n\n");
if (/^import /m.test(code))
  throw new Error("Unresolved import in offline bundle");
const runtimeCode = files.filter(n=>!["main","presentation"].includes(n)).map(n=>fs.readFileSync(path.join(root,"src",n+".js"),"utf8").replace(/^import[\s\S]*?;\s*/gm,"").replace(/\bexport (?=(?:class|function|const|let)\b)/g, "")).join("\n");
const plannerCode=(runtimeCode+"\n"+fs.readFileSync(path.join(root,"src","planning-worker.js"),"utf8").replace(/^import[\s\S]*?;\s*/gm,"")).replaceAll('import.meta.url','self.location.href');
const workerCode = ("globalThis.__warehousePlannerSource = "+JSON.stringify(plannerCode)+";\n"+runtimeCode + "\n" + fs.readFileSync(path.join(root,"src","simulation-worker.js"),"utf8").replace(/^import[\s\S]*?;\s*/gm,"")).replaceAll('import.meta.url','self.location.href');
new Script(plannerCode,{filename:'offline-planning-worker.js'});
new Script(workerCode,{filename:'offline-simulation-worker.js'});
new Script(code.replaceAll('import.meta.url','document.baseURI'),{filename:'offline-dashboard.js'});
let html = fs.readFileSync(path.join(root, "index.html"), "utf8");

html = html.replace(
  /<link rel="stylesheet" href="style\.css"\s*\/?\s*>/,
  () => `<style>${fs.readFileSync(path.join(root, "style.css"), "utf8")}</style>`,
);
html = html.replace(
  '<script type="module" src="src/main.js"></script>',
  () => `<div id="startup-status" role="status" style="position:fixed;bottom:18px;left:18px;right:18px;padding:14px 18px;background:#fff4dd;color:#604219;border:1px solid #d5b477;border-radius:6px;z-index:1000">Loading simulator…</div>
<noscript>This simulator needs JavaScript. Save the HTML file and open it directly in Chrome or Edge, rather than an attachment preview.</noscript>
<script>
(() => {
  const panel=document.getElementById('startup-status');
  let ready=false,failed=false;
  globalThis.__warehouseStartupReady=()=>{ready=true;if(!failed)panel.hidden=true;};
  globalThis.__warehouseStartupError=message=>{failed=true;panel.hidden=false;panel.textContent='Simulator could not start: '+message+' Save this file and open it directly in Chrome or Edge. Attachment previews may block scripts or workers.';};
  window.addEventListener('error',e=>globalThis.__warehouseStartupError(e.message||'Script execution was blocked.'));
  window.addEventListener('unhandledrejection',e=>globalThis.__warehouseStartupError(e.reason?.message||String(e.reason)));
  setTimeout(()=>{if(!ready&&!failed)globalThis.__warehouseStartupError('No response from the simulation worker.');},12000);
})();
</script>
<script>\nglobalThis.__warehouseWorkerSource = ${JSON.stringify(workerCode).replaceAll('<','\\u003c')};\n${code.replaceAll('import.meta.url','document.baseURI').replaceAll('</script','<\\/script')}\n</script>`,
);
fs.writeFileSync(path.join(root, "Open Warehouse Lab.html"), html);
console.log(
  "Built Open Warehouse Lab.html (no server or installation required).",
);
