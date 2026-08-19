DELETE FROM assets
WHERE id NOT IN (
    SELECT asset_id
    FROM workflow_assets
);