# Book Personalization Skill

Personalize book chapter content based on reader skill level.

## Usage

```
/book-personalize <chapter-file-path> <level>
```

Where `<level>` is one of: `beginner`, `intermediate`, `advanced`

## Instructions

You are an expert technical educator. Adapt chapter content to match the reader's experience level.

1. **Read** the chapter file at the given path.

2. **Analyze** the content and identify:
   - Core concepts that all levels need
   - Advanced topics that beginners can skip
   - Areas where additional explanation helps beginners
   - Areas where deeper dives benefit advanced readers

3. **Personalize** based on the target level:

### Beginner Level
- Add prerequisite explanations before complex topics
- Include step-by-step breakdowns of code examples
- Add analogies and real-world comparisons
- Simplify jargon with plain-language descriptions
- Add "Before you continue" prerequisite boxes
- Include more visual descriptions and diagrams references

### Intermediate Level
- Keep the content mostly as-is
- Add "Pro Tips" for optimization and best practices
- Include links to deeper resources
- Add comparison tables for alternative approaches

### Advanced Level
- Remove basic explanations
- Add performance optimization notes
- Include research paper references
- Add architecture discussion and design tradeoffs
- Include production deployment considerations
- Add benchmarking and profiling guidance

4. **Write** the personalized version maintaining all Markdown formatting.

## Output

The personalized chapter content written to stdout or a specified output file. Original file is never modified.
