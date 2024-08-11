# ROSA Project Governance

This governance model aims to create an open source community that encourages transparency, contributions, and collaboration, but maintains sound technical and quality standards. Our goal is to build a community comprised of members across the AI/ML + Robotics communities and beyond, including from private organizations, universities, government organizations, and international organizations. 

The project follows a fairly liberal contribution model where people and/or organizations who do the most work will have the most influence on project direction. Roles determine decision making influence, and governing committees (e.g. technical steering, project steering) are set up to ensure the project's direction is in-line with requirements / goals while supporting flexibility for future growth and membership. Technical decision making will primarily be made through a "[consensus-seeking](https://en.wikipedia.org/wiki/Consensus-seeking_decision-making)" approach within the respective governing committees. 

## Roles

| Role                                | Restricted To | Description                                                                                                                                                                           | Read/Clone                           | Propose Pull Request                 | Comment in Tickets / Discussions     | Triage                               | Review                               | Commit                               | Technical Decisions                  | Project Decisions                    |
| ----------------------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------ | ------------------------------------ | ------------------------------------ | ------------------------------------ | ------------------------------------ | ------------------------------------ | ------------------------------------ |
| User                                | None          | Anyone downloading, deploying, or operating the software to meet a specific objective.                                                                                                | ✅ | ✅ | ✅ | ❌    | ❌    | ❌    | ❌    | ❌    |
| Contributor                         | None          | Anyone providing input to the project, including: code, issues, documentation, graphics, etc.                                                                                         | ✅ | ✅ | ✅ | ❌    | ❌    | ❌    | ❌    | ❌    |
| Triager                             | Contributor   | Subset of contributors demonstrating a strong familiarity with the project.                                                                                                           | ✅ | ✅ | ✅ | ✅ | ❌    | ❌    | ❌    | ❌    |
| Collaborator                        | Contributor   | Subset of contributors granted write access to one or more of the project repositories upon selection by TSC                                                                          | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ❌    | ❌    |
| Technical Steering Committee Member | Collaborator  | A subset of collaborators having technical decision making authority and admin privileges                                                                                             | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ❌    |
| Project Steering Committee Member   | Stakeholders  | Sponsor organization representatives (i.e. those providing funding to the project) and key stakeholders with authority to guide project based on requirements, budget, schedule, etc. | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Product Manager                     | Stakeholders  | Overall manager of project with final authority over all key decisions when consensus cannot be reached                                                                                    | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |

### User

Anyone who has downloaded, deployed, or operated ROSA to meet a specific objective. This project was primarily designed for augmenting the robot developer experience using advanced AI agents, but let us know if you've found other uses for it.  

### Contributor

Contributors include anyone that provides input to the project. This includes code, issues, documentation, graphics, designs, or anything else that tangibly improves the project. We encourage you to start contributing right away by submitting an [Issue]([INSERT LINK TO ISSUE TRACKING SYSTEM]). 

### Triager

Subset of contributors who have demonstrated a strong familiarity with the project and are regularly contributing to the project via issue creation, commenting, discussions, etc. Triagers are given specific permissions do the following:

    - Label issues and pull requests
    - Comment, close, and reopen issues and pull requests
 
### Collaborator

Subset of contributors who have been given write access to one or more project repositories. Both contributors and collaborators can propose changes to the project via pull requests, but only collaborators can formally review and approve (merge) these requests. Any contributor who has made a non-trivial contribution should be on-boarded as a collaborator in a timely manner. 

If you are planning on making a substantial contribution to the project or feel as though you should be given write access to a repository, please send a request to [@RobRoyce](https://github.com/RobRoyce) ([email](mailto:01-laptop-voiced@icloud.com))

### Technical Steering Committee Member

A subset of the collaborators forms the Technical Steering Committee (TSC). The TSC has authority over the following aspects of this project:

- Technical direction and guidance
- Committee governance and process 
- Contribution policy
- Conduct guidelines
- Maintaining the list of collaborators

#### TSC Committee Members
- [@RobRoyce](https://github.com/RobRoyce) ([email](mailto:01-laptop-voiced@icloud.com)), NASA Jet Propulsion Laboratory



<details>

<summary>Emeriti</summary>

#### TSC Emeriti
- None

</details>
 
#### Scope

The TSC is primarily responsible for the following project repositories:

- [ROSA](https://github.com/nasa-jpl/rosa)
- [CORTEX](https://github.com/nasa-jpl/cortex)

However, the TSC also has the responsibility to interface with and monitor third-party dependencies of the project for key changes impacting ROSA.


#### Decision Making Process

Any community member can create an issue or comment asking the TSC to review something. Prior to implementing a substantial contribution, the design of that contribution should be reviewed by at least one member of the TSC. If consensus-seeking fails during the review of a pull request or in design discussions, the issue will be addressed by the TSC to make a determination on direction. TSC members will meet regularly and will keep track of decisions made when consensus was not met. 

The TSC can nominate new members at any time. Candidates for membership tend to be collaborators who have shown great dedication to the project and/or experts in a particular domain or project component. TSC members are expected to be active contributors or members who have made significant contributions within the past 12 months. 

### Project Management Committee (PMC) Member

The Project Management Committee (PMC) is made up of sponsor organization representatives (i.e. those providing funding to the project) and key stakeholders who rely or plan to rely on the project to meet a critical need. The PMC has the following responsibilities:

- Maintaining the overall project roadmap
- Determining project requirements and commitments to sponsors and stakeholders
- Assessing available resources and allocating them across the project
- Monitoring and reporting on progress against the roadmap 
- On-boarding new sponsors and stakeholders
- Overall project governance (including this policy)
- Addressing any legal considerations

#### PMC Committee Members
- [@RobRoyce](https://github.com/RobRoyce) ([email](mailto:01-laptop-voiced@icloud.com)), NASA Jet Propulsion Laboratory

<details>

<summary>Emeriti</summary>

#### PMC Emeriti
[//]: # (- [INSERT MEMBER NAME] &#40;[username1]&#40;[INSERT LINK TO USERNAME]&#41;, [INSERT ORG ASSOCIATION])
- None

</details>   

#### Scope

The PMC has management authority over the same project repositories over which the TSC has technical authority over.   

#### Decision Making Process

The PMC will consist of a product manager and additional representative from sponsors and key stakeholders. The PMC or sponsoring organizations can nominate new members at any time. Care should be taken not to include too many members from a single stakeholder project or organization.

Regular stakeholder meetings are held to discuss current project status and propose changes to the project roadmap. If stakeholder representatives and sponsors concur with these proposals during the meeting, they will be immediately adopted. A member of the PMC will ensure the changes have been captured and recorded. Regular stakeholder meetings will be open to the entire community, but only members of the PMC have decision making authority. 

Additional meetings may be held if consensus is not met or to discuss significant changes to the roadmap due to changes in stakeholder priorities or available resources. Any decision made outside of stakeholder meetings must still be approved by all sponsors and stakeholders. If full consensus cannot be made, the product manager has the final authority to determine project direction. Any non-concurrences from stakeholders or sponsors will be noted. 

### Product Manager

Overall manager of the project with final authority over all key decisions when consensus cannot be reached within the TSC or PSC. The product manager is often chosen at the onset of project initiation and can be reassigned by the PMC (with institutional approval, if applicable).

# Acknowledgements

Much of this governance model was adapted from the other notable open source projects including [node.js](https://github.com/nodejs/node/blob/main/GOVERNANCE.md), [OpenSSL](https://www.openssl.org/policies/omc-bylaws.html), [PostgresQL](https://www.postgresql.org/developer/), and [OpenMCT](https://github.com/nasa/openmct/blob/master/CONTRIBUTING.md). We would like to thank those projects for setting the foundation upon which this model was built.