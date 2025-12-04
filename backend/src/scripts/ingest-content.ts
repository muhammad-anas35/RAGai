import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';

/**
 * Content reader for ingesting Docusaurus markdown files
 * Reads all .md and .mdx files from the docs directory
 */

export interface DocumentMetadata {
    path: string;
    title: string;
    chapter?: string;
    section?: string;
    content: string;
    wordCount: number;
}

/**
 * Read all markdown files from a directory recursively
 */
export async function readDocuments(docsDir: string): Promise<DocumentMetadata[]> {
    const documents: DocumentMetadata[] = [];

    async function readDir(dir: string): Promise<void> {
        const entries = await fs.promises.readdir(dir, { withFileTypes: true });

        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);

            if (entry.isDirectory()) {
                // Recursively read subdirectories
                await readDir(fullPath);
            } else if (entry.isFile() && (entry.name.endsWith('.md') || entry.name.endsWith('.mdx'))) {
                // Read markdown file
                const content = await fs.promises.readFile(fullPath, 'utf-8');
                const { data: frontmatter, content: markdownContent } = matter(content);

                // Extract metadata
                const relativePath = path.relative(docsDir, fullPath);
                const pathParts = relativePath.split(path.sep);

                // Try to extract chapter and section from path
                let chapter: string | undefined;
                let section: string | undefined;

                if (pathParts.length > 1) {
                    // e.g., physical-ai-book/chapter1/foundations.md
                    if (pathParts[0] === 'physical-ai-book' && pathParts.length > 2) {
                        chapter = pathParts[1]; // chapter1, chapter2, etc.
                        section = pathParts[2].replace(/\.(md|mdx)$/, '');
                    }
                }

                // Extract title from frontmatter or first heading
                let title = frontmatter.title || frontmatter.sidebar_label || '';
                if (!title) {
                    const headingMatch = markdownContent.match(/^#\s+(.+)$/m);
                    title = headingMatch ? headingMatch[1] : path.basename(entry.name, path.extname(entry.name));
                }

                // Calculate word count
                const wordCount = markdownContent.split(/\s+/).filter(word => word.length > 0).length;

                documents.push({
                    path: relativePath,
                    title,
                    chapter,
                    section,
                    content: markdownContent,
                    wordCount
                });
            }
        }
    }

    await readDir(docsDir);
    return documents;
}

/**
 * Read a single document by path
 */
export async function readDocument(filePath: string): Promise<DocumentMetadata | null> {
    try {
        const content = await fs.promises.readFile(filePath, 'utf-8');
        const { data: frontmatter, content: markdownContent } = matter(content);

        const title = frontmatter.title || frontmatter.sidebar_label || path.basename(filePath, path.extname(filePath));
        const wordCount = markdownContent.split(/\s+/).filter(word => word.length > 0).length;

        return {
            path: filePath,
            title,
            content: markdownContent,
            wordCount
        };
    } catch (error) {
        console.error(`Error reading document ${filePath}:`, error);
        return null;
    }
}

/**
 * Filter documents by chapter
 */
export function filterByChapter(documents: DocumentMetadata[], chapter: string): DocumentMetadata[] {
    return documents.filter(doc => doc.chapter === chapter);
}

/**
 * Get document statistics
 */
export function getDocumentStats(documents: DocumentMetadata[]) {
    const totalDocs = documents.length;
    const totalWords = documents.reduce((sum, doc) => sum + doc.wordCount, 0);
    const chapters = new Set(documents.map(doc => doc.chapter).filter(Boolean));

    return {
        totalDocuments: totalDocs,
        totalWords,
        averageWordsPerDocument: totalDocs > 0 ? Math.round(totalWords / totalDocs) : 0,
        chapters: Array.from(chapters).sort()
    };
}
