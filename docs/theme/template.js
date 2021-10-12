const tmpl = module.exports = {
// article layout ... used by other templates 
page(data) {
  return `<!doctype html>
<html class="theme-light">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, minimum-scale=1, user-scalable=no">
<meta name="description" content="${data.description || (data.title + ' - microjam page')}">
${data.date ? `<meta name="date" content="${new Date(data.date).toString()}">` : ''}
${data.tags ? `<meta name="keywords" content="${data.tags.join()}">` : ''}
<title>${data.title}</title>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/highlight.js@9.18.1/styles/vs2015.min.css">
${data.math ? `<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex/dist/katex.min.css">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/markdown-it-texmath/css/texmath.css">` : ''}
<link rel="stylesheet" href="./theme/styles.css">
</head>
<body id="top">
<header>
  <a class="left" href="./introduction.html"></a>
  <div class="right">
    <h1>${data.title}</h1>
    <h3 style="font-size: 18px; margin-top:0">${data.subtitle}</h3>
  </div>
</header>

<main>
  <nav>
     <h2 style="margin: 0.5em"><a href="#top"></a></h2>
     ${data.uses && data.uses.find((use) => use.uri === 'navigation.md').content || 'no navigation data !'}
  </nav>
  <article>
  <div style="display: flex; align-items: center; flex-direction: column;">
    <h4 style="margin: 0.5em">${data.authors.join()}</h4>
    <h5 style="margin: 0.5em">${data.adresses.join()}</h5>
    <h5 style="margin: 0.5em">${data.date}</h5>
    <h5 style="margin: 0.5em"><b>Keywords:</b> ${data.tags.join(", ")}</h5>
  </div>
  ${data.content}
  </article>
</main>
<footer>
  <span class="left">&copy; Documentation</span>
  <span class="center">powered by &mu;Jam &amp; <a href="https://code.visualstudio.com/">VSCode</a> &mdash; hosted by <a href="https://github.com/">GitHub</a></span>
  <span class="right"
        title="toggle light/dark theme"
        onclick="document.documentElement.className = document.documentElement.className === 'theme-dark' ? 'theme-light' : 'theme-dark';">
    &#9788;
  </span>
</footer>
</body>
</html>`
}

}
