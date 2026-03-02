# Book Chapter Writing Skill

Generate or expand a chapter for the Physical AI & Humanoid Robotics textbook.

## Usage

```
/book-chapter <chapter-topic>
```

## Instructions

You are writing a chapter for a technical textbook on Physical AI and Humanoid Robotics. Follow these steps:

1. **Research**: Use the Task tool with an Explore agent to find existing chapter content in `physical-ai-book/docs/` and understand the book's structure, tone, and formatting conventions.

2. **Outline**: Generate a detailed chapter outline with:
   - Learning objectives (3-5 per chapter)
   - Section headings with brief descriptions
   - Code examples planned (Python, ROS 2, or simulation code)
   - Key concepts and terminology

3. **Write**: Generate the full chapter in Markdown following these conventions:
   - Use `##` for main sections, `###` for subsections
   - Include practical code examples with explanations
   - Add "Key Takeaway" callout boxes using Docusaurus admonitions (`:::tip`)
   - Include exercises at the end of each section
   - Reference other chapters where relevant using relative links

4. **Verify**: Use a Task agent to verify:
   - All code examples are syntactically correct
   - Technical claims are accurate
   - Links to other chapters are valid
   - Markdown renders correctly

## Output

Write the chapter file to `physical-ai-book/docs/<chapter-slug>.md` with proper Docusaurus frontmatter:

```yaml
---
sidebar_position: <number>
title: "<Chapter Title>"
description: "<One-line description>"
---
```

## Quality Standards

- Target 2000-4000 words per chapter
- Include at least 3 code examples
- Every technical term should be defined on first use
- Balance theory with practical application
