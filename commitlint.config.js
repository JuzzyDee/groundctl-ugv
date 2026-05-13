// commitlint config — enforces Conventional Commits format on commit messages.
// See CONTRIBUTING.md for the full convention.
//
// To enable: `npm install --save-dev @commitlint/cli @commitlint/config-conventional husky`
// then `npx husky add .husky/commit-msg 'npx --no -- commitlint --edit "$1"'`
//
// Or use a lightweight bash hook in .git/hooks/commit-msg if you don't want
// the Node dependency. The format below is the source of truth either way.

module.exports = {
  extends: ['@commitlint/config-conventional'],
  rules: {
    'type-enum': [
      2,
      'always',
      ['feat', 'fix', 'docs', 'chore', 'refactor', 'perf', 'test', 'ci'],
    ],
    'scope-enum': [
      2,
      'always',
      [
        'bridge',
        'camera',
        'intent',
        'perception',
        'motor',
        'ui',
        'firmware',
        'docs',
        'infra',
        // allow empty scope (chore-level commits affecting the whole repo)
        '',
      ],
    ],
    'subject-case': [2, 'never', ['upper-case', 'pascal-case', 'start-case']],
    'subject-empty': [2, 'never'],
    'subject-full-stop': [2, 'never', '.'],
    'header-max-length': [2, 'always', 100],
    'body-leading-blank': [1, 'always'],
    'footer-leading-blank': [1, 'always'],
  },
};
