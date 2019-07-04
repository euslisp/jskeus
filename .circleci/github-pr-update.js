#!/usr/bin/env node

const bot = require("./circle-github-bot/").create();

bot.comment(process.env.GH_AUTH_TOKEN, `
Thank you for contributing jskeus documentation<br>
Please check latest documents before merging<br>

PDF version of Japanese jmanual: <strong>${bot.artifactLink('artifacts/jmanual.pdf', 'jmanual.pdf')}</strong>
HTML version of Japanese manual: <strong>${bot.artifactLink('artifacts/html/jmanual.html', 'jmanual.html')}</strong>
Sphinx (ReST) version of Japanese manual: <strong>${bot.artifactLink('artifacts/rst/html/jmanual.html', 'jmanual.rst')}</strong>
`);
