from fastapi import APIRouter, HTTPException, Depends
from typing import List
from uuid import UUID
import uuid
from datetime import datetime
from ...models.text_selection import TextSelection, TextSelectionCreate
from ...config.database import get_db_session
from sqlalchemy.orm import Session


router = APIRouter()


@router.post("/", response_model=TextSelection, status_code=201)
async def create_text_selection(
    text_selection: TextSelectionCreate,
    db: Session = Depends(get_db_session)
):
    """
    Create a new text selection
    """
    try:
        # Validate input
        if len(text_selection.content) < 10:
            raise HTTPException(
                status_code=400,
                detail="Selected text must be at least 10 characters long"
            )

        if len(text_selection.content) > 5000:
            raise HTTPException(
                status_code=400,
                detail="Selected text exceeds maximum length of 5000 characters"
            )

        if text_selection.start_position >= text_selection.end_position:
            raise HTTPException(
                status_code=400,
                detail="Start position must be less than end position"
            )

        # Create text selection object with generated ID and timestamp
        db_text_selection = TextSelection(
            id=uuid.uuid4(),
            content=text_selection.content,
            start_position=text_selection.start_position,
            end_position=text_selection.end_position,
            book_section=text_selection.book_section,
            user_id=text_selection.user_id,
            created_at=datetime.utcnow()
        )

        # In a real implementation, you would save to the database here
        # db.add(db_text_selection)
        # db.commit()
        # db.refresh(db_text_selection)

        return db_text_selection

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error creating text selection: {str(e)}"
        )


@router.get("/{selection_id}", response_model=TextSelection)
async def get_text_selection(
    selection_id: str,
    db: Session = Depends(get_db_session)
):
    """
    Get a specific text selection by ID
    """
    try:
        selection_uuid = UUID(selection_id)

        # In a real implementation, you would fetch from the database here
        # db_text_selection = db.query(TextSelection).filter(
        #     TextSelection.id == selection_uuid
        # ).first()

        # For now, return a mock response
        from datetime import datetime
        import uuid

        return TextSelection(
            id=selection_uuid,
            content="This is a sample selected text for demonstration purposes.",
            start_position=100,
            end_position=200,
            book_section="chapter-1/section-1",
            user_id=None,
            created_at=datetime.utcnow()
        )

    except ValueError:
        raise HTTPException(
            status_code=400,
            detail="Invalid selection ID format"
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving text selection: {str(e)}"
        )


# Note: We'll need to import datetime for the above code to work
# Adding it at the top of the function where needed