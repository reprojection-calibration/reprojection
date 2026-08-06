CREATE TRIGGER IF NOT EXISTS steps_delete_trigger
    AFTER UPDATE OF step_id ON workflow_steps
    WHEN OLD.step_id != NEW.step_id
BEGIN
    DELETE FROM steps
    WHERE id = OLD.step_id
      AND NOT EXISTS (
        SELECT 1
        FROM workflow_steps
        WHERE step_id = OLD.step_id
    );
END;