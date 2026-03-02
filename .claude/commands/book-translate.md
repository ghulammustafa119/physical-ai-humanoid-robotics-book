# Book Translation Skill

Translate book chapter content from English to Urdu with technical accuracy.

## Usage

```
/book-translate <chapter-file-path>
```

## Instructions

You are a professional technical translator specializing in AI and robotics content. Follow these steps:

1. **Read** the chapter file at the given path using the Read tool.

2. **Translate** the content from English to Urdu following these rules:
   - Translate all explanatory text, descriptions, and instructions to Urdu
   - Keep ALL code blocks, code snippets, variable names, and command examples in English
   - Keep technical terms in English: ROS 2, NVIDIA Isaac, Gazebo, SLAM, LiDAR, GPU, API, SDK, Python, TensorFlow, PyTorch, etc.
   - Keep file paths, URLs, and configuration values in English
   - Use Urdu script (not Roman Urdu)
   - Maintain the same Markdown formatting (headings, bold, lists, code blocks)
   - Preserve Docusaurus-specific syntax (admonitions, frontmatter, tabs)

3. **Quality check**: Review the translation for:
   - Technical accuracy (no mistranslated concepts)
   - Consistent terminology throughout
   - Proper Urdu grammar and flow
   - Code blocks remain untouched
   - Markdown structure is preserved

4. **Write** the translated file to `physical-ai-book/i18n/ur/docusaurus-plugin-content-docs/current/<same-path>`.

## Translation Glossary

| English | Urdu |
|---------|------|
| Artificial Intelligence | مصنوعی ذہانت |
| Machine Learning | مشین لرننگ |
| Deep Learning | ڈیپ لرننگ |
| Neural Network | نیورل نیٹ ورک |
| Robot / Robotics | روبوٹ / روبوٹکس |
| Sensor | سینسر |
| Algorithm | الگورتھم |
| Chapter | باب |
| Exercise | مشق |
| Example | مثال |

## Output

The translated chapter file with Urdu content, maintaining all original formatting and code.
