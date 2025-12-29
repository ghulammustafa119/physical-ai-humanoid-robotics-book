// TypeScript types for book content

export interface BookContent {
  id: string;
  title: string;
  content: string;
  bookId: string;
  sectionPath: string;
  pageNumber: number;
  createdAt: string;
  updatedAt: string;
}

export interface BookSection {
  id: string;
  title: string;
  content: string;
  path: string;
  pageNumber: number;
  parentSectionId?: string;
  childSectionIds?: string[];
}

export interface BookChapter {
  id: string;
  title: string;
  sections: BookSection[];
  pageNumberStart: number;
  pageNumberEnd: number;
}