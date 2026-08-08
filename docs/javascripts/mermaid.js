(function () {
  function renderMermaid() {
    const blocks = document.querySelectorAll("pre code.language-mermaid");
    if (blocks.length === 0) return;

    blocks.forEach(function (element) {
      const pre = element.parentElement;
      const div = document.createElement("div");
      div.className = "mermaid";
      div.textContent = element.textContent;
      if (pre && pre.parentNode) {
        pre.parentNode.replaceChild(div, pre);
      }
    });

    if (typeof mermaid !== "undefined") {
      mermaid.initialize({ startOnLoad: false, theme: "default" });
      mermaid.run();
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", renderMermaid);
  } else {
    renderMermaid();
  }
})();
