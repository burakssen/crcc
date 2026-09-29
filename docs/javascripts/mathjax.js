// Preserve GitHub's $`...`$ source notation and render it in the Zensical site.
function prepareGitHubMath() {
  for (const code of document.querySelectorAll("code")) {
    const before = code.previousSibling;
    const after = code.nextSibling;
    if (
      before?.nodeType !== Node.TEXT_NODE ||
      after?.nodeType !== Node.TEXT_NODE ||
      !before.textContent.endsWith("$") ||
      !after.textContent.startsWith("$")
    ) {
      continue;
    }
    before.textContent = before.textContent.slice(0, -1);
    after.textContent = after.textContent.slice(1);
    const math = document.createElement("span");
    math.className = "arithmatex";
    math.textContent = `\\(${code.textContent}\\)`;
    code.replaceWith(math);
  }
}

window.MathJax = {
  tex: { inlineMath: [["\\(", "\\)"]] },
  options: { ignoreHtmlClass: ".*", processHtmlClass: "arithmatex" },
  startup: {
    typeset: false,
    ready() {
      MathJax.startup.defaultReady();
      document$.subscribe(() => {
        prepareGitHubMath();
        MathJax.typesetClear();
        MathJax.typesetPromise();
      });
    },
  },
};
