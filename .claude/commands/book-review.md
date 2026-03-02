# Book Content Review Skill

Review book chapter content for technical accuracy, code validity, and completeness.

## Usage

```
/book-review <chapter-file-path>
```

## Instructions

You are a technical reviewer for a Physical AI and Humanoid Robotics textbook. Perform a thorough review of the chapter.

1. **Read** the chapter file at the given path.

2. **Technical Accuracy Review**:
   - Verify all technical claims are accurate
   - Check that library/framework versions mentioned are current
   - Verify API usage matches official documentation
   - Flag any outdated information

3. **Code Review**:
   - Check all code examples for syntax errors
   - Verify imports and dependencies are correct
   - Test that code snippets would run as-is (or note missing context)
   - Check for security issues (hardcoded credentials, unsafe practices)
   - Verify code follows Python/ROS 2 best practices

4. **Content Quality Review**:
   - Check for clear learning progression within the chapter
   - Verify all technical terms are defined on first use
   - Check for consistency in terminology
   - Verify cross-references to other chapters are valid
   - Check that exercises match the content covered

5. **Formatting Review**:
   - Verify Docusaurus frontmatter is complete and correct
   - Check Markdown formatting renders correctly
   - Verify code block language tags are correct
   - Check image references point to existing files
   - Verify admonitions use correct Docusaurus syntax

## Output

Generate a review report in this format:

```markdown
# Review: <Chapter Title>

## Summary
<1-2 sentence overall assessment>

## Issues Found

### Critical (must fix)
- [ ] <issue description> (line X)

### Important (should fix)
- [ ] <issue description> (line X)

### Minor (nice to fix)
- [ ] <issue description> (line X)

## Strengths
- <what the chapter does well>

## Recommendations
- <suggestions for improvement>
```

Write the review to `specs/reviews/<chapter-slug>-review.md`.
