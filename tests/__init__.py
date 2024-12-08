2024/12/08 06:01:18
190887183
rule Over Ruled started
Root=1-6756253d-051c6474453874777706de20
{
	"installId": "58143362",
	"updatedAt": "2024-12-08T23:00:57.603Z",
	"createdAt": "2024-12-08T23:00:57.603Z",
	"ruleSet": {
		"automatedSecurityFixes": {
			"enabled": false
		},
		"vulnerabilityAlerts": {
			"enabled": false
		},
		"repository": {
			"has_projects": false,
			"allow_squash_merge": false,
			"allow_merge_commit": false,
			"has_wiki": false,
			"visibility": "internal",
			"is_template": false,
			"default_branch": "Maintenance",
			"delete_branch_on_merge": true,
			"allow_rebase_merge": false,
			"has_issues": false
		},
		"branchProtections": {
			"unprotectedBranches": [
				"Master"
			],
			"protectedBranches": [
				{
					"branchName": "$default",
					"protectionSetting": {
						"required_pull_request_reviews": {
							"dismissal_restrictions": {},
							"required_approving_review_count": 5,
							"require_code_owner_reviews": true,
							"dismiss_stale_reviews": true
						},
						"required_status_checks": {
							"strict": true,
							"contexts": [
								"testing-production"
							]
						},
						"required_linear_history": true,
						"restrictions": null,
						"enforce_admins": true,
						"allow_deletions": true,
						"allow_force_pushes": true
					},
					"enforceCommitSignature": true
				}
			]
		},
		"teamsSetting": {
			"teamPermissions": [],
			"removeOthers": true
		},
		"topics": {
			"removeOthers": true,
			"names": [
				"Owner",
				"Showcase"
			]
		}
	},
	"id": "d47aad60-0e2f-4ccd-88b8-a2b6235d26b4",
	"name": "Over Ruled",
	"matcher": {
		"matchType": "regex",
		"regex": ".*"
	}
}
